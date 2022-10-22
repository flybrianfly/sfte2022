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

#include <array>
#include "ams5915.h"
#include "statistics.h"
#include "filter.h"
#include "airdata.h"
#include "checksum.h"
#include "polytools.h"

/*
* This code samples a static and differential pressure transducer, low
* pass filters the data, uses it to estimate airspeed and pressure altitude,
* and transmits this data to PDAS over a wireless radio modem. Potentiometers,
* are included to mimic data from angle of attack and flank angle vanes.
*/

/* Whether or not to print data to the screen */
const bool DEBUG = false;

/* Pressure transducers */
bfs::Ams5915 static_pres(&Wire, 0x10, bfs::Ams5915::AMS5915_1200_B);
bfs::Ams5915 diff_pres(&Wire, 0x11, bfs::Ams5915::AMS5915_0020_D);

/* Running statistics method to estimate differential pressure turn-on bias */
bfs::RunningStats<float> diff_pres_bias_stats;

/* Timer for estimating diff pres bias */
elapsedMillis t_ms;
const int16_t CAL_TIME_MS = 5000;

/* Diff pres bias */
float diff_pres_bias_pa;

/* Low pass filters */
const float STATIC_PRES_CUTOFF_HZ = 2;
const float DIFF_PRES_CUTOFF_HZ = 5;
bfs::Iir<float> static_pres_dlpf;
bfs::Iir<float> diff_pres_dlpf;

/* Sampling rate */
const int16_t SAMPLING_RATE_HZ = 50;
const int16_t SAMPLING_PERIOD_MS = 20;

/* Airdata structure */
struct AirData {
  float static_pres_pa;
  float diff_pres_pa;
  float ias_mps;
  float pres_alt_m;
  float aoa_deg;
  float aos_deg;
};

/* PDAS external data structure */
#pragma pack(push, 1)
struct PdasData {
  uint8_t header[2] = {0x42, 0x46};
  uint8_t starting_single_idx;
  uint8_t starting_double_idx;
  uint8_t num_singles;
  uint8_t num_doubles;
  AirData air_data;
  uint16_t checksum;
} pdas_data;
#pragma pack(pop)

/* Fletcher-16 checksum */
bfs::Fletcher16 chk;

/* Analog Input */
const int8_t AOA_PIN = 24;
const int8_t AOS_PIN = 25;
const int ANALOG_RESOLUTION_BITS = 12;
const float VOLTAGE_RANGE = 3.3;
const float ANALOG_COUNT_RANGE = std::pow(2.0f, ANALOG_RESOLUTION_BITS) - 1.0f;
const float AIN_VOLTAGE_SCALE = VOLTAGE_RANGE / ANALOG_COUNT_RANGE;

/* Polyfit from voltage to angle */
std::array<float, 2> aoa_polyvals = {12.1212f, -20.0f};
std::array<float, 2> aos_polyvals = {6.0606f, -10.0f};

void setup() {
  /* Debug to print air data values */
  if (DEBUG) {
    Serial.begin(115200);
    while (!Serial) {}
  }
  /* Wireless radio modem */
  Serial7.begin(115200);
  /* Analog setup */
  analogReadResolution(ANALOG_RESOLUTION_BITS);
  /* Start communication with pressure transducers */
  Wire.begin();
  Wire.setClock(400000);
  if (!static_pres.Begin()) {
    Serial.println("Error initializing static pressure transducer");
    while (1) {}
  }
  if (!diff_pres.Begin()) {
    Serial.println("Error initializing diff pressure transducer");
    while (1) {}
  }
  /* Estimate diff pres bias */
  t_ms = 0;
  while (t_ms < CAL_TIME_MS) {
    if (diff_pres.Read()) {
      diff_pres_bias_stats.Update(diff_pres.pres_pa());
    }
  }
  diff_pres_bias_pa = diff_pres_bias_stats.mean();
  /* Initialize low pass filters */
  while (!static_pres.Read()) {}
  static_pres_dlpf.Init(STATIC_PRES_CUTOFF_HZ, SAMPLING_RATE_HZ,
                        static_pres.pres_pa());
  while (!diff_pres.Read()) {}
  diff_pres_dlpf.Init(DIFF_PRES_CUTOFF_HZ, SAMPLING_RATE_HZ,
                      diff_pres.pres_pa() - diff_pres_bias_pa);
  /* Setup the PDAS data packet */
  pdas_data.starting_single_idx = 0;
  pdas_data.starting_double_idx = 0;
  pdas_data.num_singles = 6;
  pdas_data.num_doubles = 0;
}

void loop() {
  if (t_ms > SAMPLING_PERIOD_MS) {
    t_ms = 0;
    /* Get the sensor data and update the state estimation */
    if (static_pres.Read()) {
      pdas_data.air_data.static_pres_pa = static_pres_dlpf.Filter(static_pres.pres_pa());
      pdas_data.air_data.pres_alt_m = bfs::PressureAltitude_m(pdas_data.air_data.static_pres_pa);
    }
    if (diff_pres.Read()) {
      pdas_data.air_data.diff_pres_pa = diff_pres_dlpf.Filter(diff_pres.pres_pa() - diff_pres_bias_pa);
      pdas_data.air_data.ias_mps = bfs::Ias_mps(pdas_data.air_data.diff_pres_pa);
    }
    /* Angle of attach and flank angle POTs */
    pdas_data.air_data.aoa_deg = bfs::polyval(aoa_polyvals.data(), aoa_polyvals.size(), static_cast<float>(analogRead(AOA_PIN)) * AIN_VOLTAGE_SCALE);
    pdas_data.air_data.aos_deg = bfs::polyval(aos_polyvals.data(), aos_polyvals.size(), static_cast<float>(analogRead(AOS_PIN)) * AIN_VOLTAGE_SCALE);
    /* Compute the checksum */
    pdas_data.checksum = chk.Compute((uint8_t *)&pdas_data, sizeof(pdas_data) - 2);
    Serial7.write((uint8_t *)&pdas_data, sizeof(pdas_data));
    /* Print the air data */
    if (DEBUG) {
      Serial.print(pdas_data.air_data.static_pres_pa);
      Serial.print("\t");
      Serial.print(pdas_data.air_data.diff_pres_pa);
      Serial.print("\t");
      Serial.print(pdas_data.air_data.pres_alt_m);
      Serial.print("\t");
      Serial.print(pdas_data.air_data.ias_mps);
      Serial.print("\n");
    }
  }
}
