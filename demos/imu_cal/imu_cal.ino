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
#include "eigen.h"
#include "statistics.h"
#include "EEPROM.h"

/*
* This code depicts the benefits of calibrating an IMU. Gyro turn-on biases
* are estimated and removed. Accel bias and scale factor are loaded from
* EEPROM and applied.
*/

/* Mpu6500 object, SPI bus, CS on pin 10 */
bfs::Mpu6500 imu(&SPI, 10);

/* Vector for accel and gyro data */
Eigen::Vector3f accel_mps2, gyro_radps;

/* Matrix to rotate IMU into board frame */
Eigen::Matrix3f rotation{{0, -1, 0}, {-1, 0, 0}, {0, 0, -1}};

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
uint8_t buf[sizeof(accel_stats)];
Eigen::Matrix3f accel_scale;
Eigen::Vector3f accel_bias_mps2;

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
    accel_mps2 = accel_scale * rotation * accel_mps2 + accel_bias_mps2;
    gyro_radps = rotation * gyro_radps - gyro_bias_radps;
    Serial.print(accel_mps2[0], 4);
    Serial.print("\t");
    Serial.print(accel_mps2[1], 4);
    Serial.print("\t");
    Serial.print(accel_mps2[2], 4);
    Serial.print("\t");
    Serial.print(gyro_radps[0], 4);
    Serial.print("\t");
    Serial.print(gyro_radps[1], 4);
    Serial.print("\t");
    Serial.print(gyro_radps[2], 4);
    Serial.print("\n");
  }
}
