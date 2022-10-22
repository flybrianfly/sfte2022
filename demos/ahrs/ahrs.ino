/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "mpu6500.h"
#include "lis3mdl.h"
#include "uNavAHRS.h"
#include "EEPROM.h"
#include "statistics.h"
#include "units.h"

/*
* This code implemented a 7 state EKF to estimate IMU attitude, heading
* and gyro biases. The gyro turn-on biases are estimated and removed. Accel
* and mag bias and scale factors are loaded from EEPROM and removed. Sensors
* are rotated into the board frame and input to the EKF. The EKF takes
* two minutes to estimate covariance before outputting data.
*/

/* Mpu6500 object, SPI bus, CS on pin 10 */
bfs::Mpu6500 imu(&SPI, 10);

/* LIS3MDL object, SPI bus, CS on pin 42 */
bfs::Lis3mdl mag(&SPI, 42);

/* uNavAHRS object */
uNavAHRS Filter;

/* Vector for accel gyro and mag data */
Eigen::Vector3f accel_mps2, gyro_radps, mag_ut;

/* Matrix to rotate IMU and mag into board frame */
Eigen::Matrix3f rotation{{0, -1, 0}, {-1, 0, 0}, {0, 0, -1}};
Eigen::Matrix3f mag_board_rotation{{-1, 0, 0}, {0, -1, 0}, {0, 0, -1}};

/* Timer for estimating gyro bias */
elapsedMillis t_ms;
const int16_t CAL_TIME_MS = 5000;

/* Gyro bias stats */
bfs::RunningStats<float> gx, gy, gz;
Eigen::Vector3f gyro_bias_radps;

/* Accel stats */
struct AccelStats {
  float accel_bias_mps2[3];
  float accel_scale[3];
} accel_stats;
Eigen::Matrix3f accel_scale;
Eigen::Vector3f accel_bias_mps2;

/* Mag stats */
struct MagStats {
  float mag_bias_ut[3];
  float mag_scale[3];
} mag_stats;

uint8_t buf[sizeof(accel_stats)];

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  /* Start the SPI bus */
  pinMode(10, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(42, OUTPUT);
  digitalWriteFast(10, HIGH);
  digitalWriteFast(45, HIGH);
  digitalWriteFast(42, HIGH);
  SPI.begin();
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }
  /* Set the DLPF */
  if (!imu.ConfigDlpfBandwidth(bfs::Mpu6500::DLPF_BANDWIDTH_20HZ)) {
    Serial.println("Error configuring DLPF");
    while(1) {}
  }
  /* Initialize and configure mag */
  if (!mag.Begin()) {
    Serial.println("Error initializing communication with mag");
    while(1) {}
  }
  if (!mag.ConfigOdr(bfs::Lis3mdl::ODR_155HZ)) {
    Serial.println("Error configuring mag ODR");
    while(1) {}
  }
  /* Calibrate the gyro */
  t_ms = 0;
  while (t_ms < CAL_TIME_MS) {
    if (imu.Read()) {
      gyro_radps[0] = imu.gyro_x_radps();
      gyro_radps[1] = imu.gyro_y_radps();
      gyro_radps[2] = imu.gyro_z_radps();
      gyro_radps = rotation * gyro_radps;
      gx.Update(gyro_radps[0]);
      gy.Update(gyro_radps[1]);
      gz.Update(gyro_radps[2]);
    }
  }
  gyro_bias_radps[0] = gx.mean();
  gyro_bias_radps[1] = gy.mean();
  gyro_bias_radps[2] = gz.mean();
  /* Load the accel stats from EEPROM */
  for (size_t i = 0; i < sizeof(buf); i++) {
    buf[i] = EEPROM.read(i);
  }
  memcpy(&accel_stats, buf, sizeof(accel_stats));
  accel_bias_mps2[0] = accel_stats.accel_bias_mps2[0];
  accel_bias_mps2[1] = accel_stats.accel_bias_mps2[1];
  accel_bias_mps2[2] = accel_stats.accel_bias_mps2[2];
  accel_scale(0, 0) = accel_stats.accel_scale[0];
  accel_scale(1, 1) = accel_stats.accel_scale[1];
  accel_scale(2, 2) = accel_stats.accel_scale[2];

  /* Load the mag stats from EEPROM */
  for (size_t i = 0; i < sizeof(buf); i++) {
    buf[i] = EEPROM.read(i + 1000);
  }
  memcpy(&mag_stats, buf, sizeof(mag_stats));
}

void loop() {
  /* Check if data read */
  if (imu.Read()) {
    accel_mps2[0] = imu.accel_x_mps2();
    accel_mps2[1] = imu.accel_y_mps2();
    accel_mps2[2] = imu.accel_z_mps2();
    gyro_radps[0] = imu.gyro_x_radps();
    gyro_radps[1] = imu.gyro_y_radps();
    gyro_radps[2] = imu.gyro_z_radps();
    gyro_radps = rotation * gyro_radps - gyro_bias_radps;
    accel_mps2 = accel_scale * rotation * accel_mps2 + accel_bias_mps2;
    if (mag.Read()) {
      if (mag.new_x_data()) {
        mag_ut[0] = mag.mag_x_ut() * mag_stats.mag_scale[0] + mag_stats.mag_bias_ut[0];
      }
      if (mag.new_y_data()) {
        mag_ut[1] = mag.mag_y_ut() * mag_stats.mag_scale[1] + mag_stats.mag_bias_ut[1];
      }
      if (mag.new_z_data()) {
        mag_ut[2] = mag.mag_z_ut() * mag_stats.mag_scale[2] + mag_stats.mag_bias_ut[2];
      }
    }
    mag_ut = mag_board_rotation * mag_ut;
    if (Filter.update(gyro_radps[0], gyro_radps[1], gyro_radps[2], accel_mps2[0], accel_mps2[1], accel_mps2[2], mag_ut[0], mag_ut[1], mag_ut[2])) {
      Serial.print(bfs::rad2deg(Filter.getPitch_rad()));
      Serial.print("\t");
      Serial.print(bfs::rad2deg(Filter.getRoll_rad()));
      Serial.print("\t");
      Serial.print(bfs::rad2deg(Filter.getHeading_rad()));
      Serial.print("\n");
    }
  }
}
