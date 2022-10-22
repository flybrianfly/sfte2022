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
#include "EEPROM.h"

/*
* This code calibrates a magnetometer by finding the min and max value
* in each axis as the magnetometer is rotated. Bias and scale factor are
* estimated for each axis.
*/

/* Mpu6500 object, SPI bus, CS on pin 10 */
bfs::Mpu6500 imu(&SPI, 10);

/* LIS3MDL object, SPI bus, CS on pin 42 */
bfs::Lis3mdl mag(&SPI, 42);

/* Used to track the highest value seen in X, Y, Z */
float hx_min, hx_max, hy_min, hy_max, hz_min, hz_max;
struct MagStats {
  float mag_bias_ut[3];
  float mag_scale[3];
} mag_stats;
uint8_t buf[sizeof(mag_stats)];


/* Timer for estimating stats */
elapsedMillis t_ms;
const int32_t CAL_TIME_MS = 60000;

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
  /* Initialize and configure mag */
  if (!mag.Begin()) {
    Serial.println("Error initializing communication with mag");
    while(1) {}
  }
  if (!mag.ConfigOdr(bfs::Lis3mdl::ODR_155HZ)) {
    Serial.println("Error configuring mag ODR");
    while(1) {}
  }
  /* Mag calibration
  * Announce starting test. Slowly rotate the board in a figure 8 to try to have
  * each axis sense the entire sphere around the sensor. The code samples
  * the mag to find the maximum and minimum values sensed in each axis. This
  * sampling takes 60 seconds to complete.
  */
  t_ms = 0;
  Serial.println("Starting Mag Calibration");
  while (t_ms < CAL_TIME_MS) {
    Serial.print(t_ms);
    Serial.print("\t");
    Serial.println(t_ms < CAL_TIME_MS);
    if (mag.Read()) {
      if (mag.new_x_data()) {
        if (mag.mag_x_ut() > hx_max) {
          hx_max = mag.mag_x_ut();
        }
        if (mag.mag_x_ut() < hx_min) {
          hx_min = mag.mag_x_ut();
        }
      }
      if (mag.new_y_data()) {
        if (mag.mag_y_ut() > hy_max) {
          hy_max = mag.mag_y_ut();
        }
        if (mag.mag_y_ut() < hy_min) {
          hy_min = mag.mag_y_ut();
        }
      }
      if (mag.new_z_data()) {
        if (mag.mag_z_ut() > hz_max) {
          hz_max = mag.mag_z_ut();
        }
        if (mag.mag_z_ut() < hz_min) {
          hz_min = mag.mag_z_ut();
        }
      }
    }
    delay(10);
  }
  /* Displays the max and min sensed in each axis */
  Serial.println("Done, results:");
  Serial.print(hx_max);
  Serial.print("\t");
  Serial.print(hx_min);
  Serial.print("\t");
  Serial.print(hy_max);
  Serial.print("\t");
  Serial.print(hy_min);
  Serial.print("\t");
  Serial.print(hz_max);
  Serial.print("\t");
  Serial.print(hz_min);
  Serial.print("\n");
  /* Finds the bias and scale factor */
  mag_stats.mag_bias_ut[0] = -(hx_max + hx_min) / 2.0f;
  mag_stats.mag_bias_ut[1] = -(hy_max + hy_min) / 2.0f;
  mag_stats.mag_bias_ut[2] = -(hz_max + hz_min) / 2.0f;
  mag_stats.mag_scale[0] = ((hx_max - hx_min) / 2.0f);
  mag_stats.mag_scale[1] = ((hy_max - hy_min) / 2.0f);
  mag_stats.mag_scale[2] = ((hz_max - hz_min) / 2.0f);
  float norm = (mag_stats.mag_scale[0] + mag_stats.mag_scale[1] + mag_stats.mag_scale[2]) / 3.0f;
  mag_stats.mag_scale[0] = norm / mag_stats.mag_scale[0];
  mag_stats.mag_scale[1] = norm / mag_stats.mag_scale[1];
  mag_stats.mag_scale[2] = norm / mag_stats.mag_scale[2];
  /* Write to EEPROM */
  memcpy(buf, &mag_stats, sizeof(buf));
  for (size_t i = 0; i < sizeof(buf); i++) {
    EEPROM.write(i + 1000, buf[i]);
  }
}

void loop() {}
