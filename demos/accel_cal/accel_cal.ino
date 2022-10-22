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
* In this code, we sample the accel in 6 orientations lined up with the
* positive and negative direction of each axis. We then are able to estimate
* bias and scale factor.
*/

/* Mpu6500 object, SPI bus, CS on pin 10 */
bfs::Mpu6500 imu(&SPI, 10);

/* Vector for accel data */
Eigen::Vector3f accel_mps2;

/* Matrix to rotate IMU into board frame */
Eigen::Matrix3f rotation{{0, -1, 0}, {-1, 0, 0}, {0, 0, -1}};

/* Timer for estimating stats */
elapsedMillis t_ms;
const int16_t CAL_TIME_MS = 5000;

/* Accel stats */
bfs::RunningStats<float> axp, ayp, azp, axn, ayn, azn;
struct AccelStats {
  float accel_bias_mps2[3];
  float accel_scale[3];
} accel_stats;
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
  /* Set the sample rate divider to get a 50 Hz rate */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configuring SRD");
    while(1) {}
  }
  /* Set the DLPF */
  if (!imu.ConfigDlpfBandwidth(bfs::Mpu6500::DLPF_BANDWIDTH_20HZ)) {
    Serial.println("Error configuring DLPF");
    while(1) {}
  }
  /* Calibrate the accel
  * 
  * Announce the desired orientation, wait 3 seconds, and then
  * sample 5 seconds of data to find the mean
  */
  Serial.println("Begin accel cal...");
  Serial.println("AZ negative");
  delay(3000);
  t_ms = 0;
  while (t_ms < CAL_TIME_MS) {
    if (imu.Read()) {
      accel_mps2[0] = imu.accel_x_mps2();
      accel_mps2[1] = imu.accel_y_mps2();
      accel_mps2[2] = imu.accel_z_mps2();
      accel_mps2 = rotation * accel_mps2;
      azn.Update(accel_mps2[2]);
    }
  }
  Serial.println("AZ positive");
  delay(3000);
  t_ms = 0;
  while (t_ms < CAL_TIME_MS) {
    if (imu.Read()) {
      accel_mps2[0] = imu.accel_x_mps2();
      accel_mps2[1] = imu.accel_y_mps2();
      accel_mps2[2] = imu.accel_z_mps2();
      accel_mps2 = rotation * accel_mps2;
      azp.Update(accel_mps2[2]);
    }
  }
  Serial.println("AX negative");
  delay(3000);
  t_ms = 0;
  while (t_ms < CAL_TIME_MS) {
    if (imu.Read()) {
      accel_mps2[0] = imu.accel_x_mps2();
      accel_mps2[1] = imu.accel_y_mps2();
      accel_mps2[2] = imu.accel_z_mps2();
      accel_mps2 = rotation * accel_mps2;
      axn.Update(accel_mps2[0]);
    }
  }
  Serial.println("AX positive");
  delay(3000);
  t_ms = 0;
  while (t_ms < CAL_TIME_MS) {
    if (imu.Read()) {
      accel_mps2[0] = imu.accel_x_mps2();
      accel_mps2[1] = imu.accel_y_mps2();
      accel_mps2[2] = imu.accel_z_mps2();
      accel_mps2 = rotation * accel_mps2;
      axp.Update(accel_mps2[0]);
    }
  }
  Serial.println("AY negative");
  delay(3000);
  t_ms = 0;
  while (t_ms < CAL_TIME_MS) {
    if (imu.Read()) {
      accel_mps2[0] = imu.accel_x_mps2();
      accel_mps2[1] = imu.accel_y_mps2();
      accel_mps2[2] = imu.accel_z_mps2();
      accel_mps2 = rotation * accel_mps2;
      ayn.Update(accel_mps2[1]);
    }
  }
  Serial.println("AY positive");
  delay(3000);
  t_ms = 0;
  while (t_ms < CAL_TIME_MS) {
    if (imu.Read()) {
      accel_mps2[0] = imu.accel_x_mps2();
      accel_mps2[1] = imu.accel_y_mps2();
      accel_mps2[2] = imu.accel_z_mps2();
      accel_mps2 = rotation * accel_mps2;
      ayp.Update(accel_mps2[1]);
    }
  }
  Serial.println("done");
  /* 
  * Find the bias and scale factor for each axis
  */
  accel_stats.accel_bias_mps2[0] = -(axp.mean() + axn.mean()) / 2.0f;
  accel_stats.accel_bias_mps2[1] = -(ayp.mean() + ayn.mean()) / 2.0f;
  accel_stats.accel_bias_mps2[2] = -(azp.mean() + azn.mean()) / 2.0f;
  accel_stats.accel_scale[0] = 9.80665f / ((axp.mean() - axn.mean()) / 2.0f);
  accel_stats.accel_scale[1] = 9.80665f / ((ayp.mean() - ayn.mean()) / 2.0f);
  accel_stats.accel_scale[2] = 9.80665f / ((azp.mean() - azn.mean()) / 2.0f);
  /* Write to EEPROM */
  memcpy(buf, &accel_stats, sizeof(buf));
  for (size_t i = 0; i < sizeof(buf); i++) {
    EEPROM.write(i, buf[i]);
  }
}

void loop() {}
