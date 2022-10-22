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
#include "units.h"
#include "statistics.h"

/*
* This code integrates gyro data at 1kHz to compute the delta angle in all
* three axis from a starting orientation. It is used to demonstrate how gyro
* data is great at estimating attitude, but suffers from drift over time.
*/

/* Mpu6500 object, SPI bus, CS on pin 10 */
bfs::Mpu6500 imu(&SPI, 10);

/* Vector for gyro data */
Eigen::Vector3f gyro_radps;

/* Delta angle */
Eigen::Vector3f delta_ang;

/* Timestep */
const float dt_s = 1e-3;

/* Sample rate divider for printing the data */
int count = 0;
const int SRD = 19;

/* Matrix to rotate IMU into board frame */
Eigen::Matrix3f rotation{{0, -1, 0}, {-1, 0, 0}, {0, 0, -1}};

/* Timer for estimating gyro bias */
elapsedMillis t_ms;
const int16_t CAL_TIME_MS = 5000;

/* Gyro bias stats */
bfs::RunningStats<float> gx, gy, gz;
Eigen::Vector3f gyro_bias_radps;

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
  if (!imu.ConfigSrd(0)) {
    Serial.println("Error configured SRD");
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
}

void loop() {
  /* Check if data read */
  if (imu.Read()) {
    /* Sample and integrate gyro data */
    gyro_radps[0] = imu.gyro_x_radps();
    gyro_radps[1] = imu.gyro_y_radps();
    gyro_radps[2] = imu.gyro_z_radps();
    gyro_radps = rotation * gyro_radps - gyro_bias_radps;
    delta_ang += gyro_radps * dt_s;
    count++;
    /* Display the data at a lower rate */
    if (count > SRD) {
      Serial.print(bfs::rad2deg(delta_ang[1]));
      Serial.print("\t");
      Serial.print(bfs::rad2deg(delta_ang[0]));
      Serial.print("\t");
      Serial.print(bfs::rad2deg(delta_ang[2]));
      Serial.print("\n");
      count = 0;
    }
  }
}
