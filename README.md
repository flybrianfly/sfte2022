# sfte2022
SFTE Symposium 2022 Workshop presentation, PDAS information, and demo code.


## Demos
Demo software is located in the *demos* folder. All software is written in Arduino to work with the BFS *FMU-R mini v1.0* UAS research flight control system, which uses a Teensy 4 MMOD processor, MPU-6500 IMU, BMP388 static pressure sensor, and LIS3MDL magnetometer.

### imu_raw
This demonstrates raw accel and gyro data from the MPU-6500, a low-cost MEMS IMU. Requires the [BFS Invensense IMU](https://github.com/bolderflight/invensense-imu) library and the [BFS Eigen](https://github.com/bolderflight/eigen/) library.

### accel_cal
Calibrates the MPU-6500 accelerometer by having the user rotate the board in all 6 directions (3-axis, up and down), using the gravity vector to excite the accel and estimating bias and scale factor for each axis. Requires the [BFS Invensense IMU](https://github.com/bolderflight/invensense-imu) library, the [BFS Eigen](https://github.com/bolderflight/eigen/) library and the [BFS Statistics](https://github.com/bolderflight/statistics) library.

### imu_cal
This demonstrates calibrated data from the MPU-6500. The gyro is calibrated during startup and the accel calibration factors from the *accel_cal* code are loaded from EEPROM. Requires the [BFS Invensense IMU](https://github.com/bolderflight/invensense-imu) library, the [BFS Eigen](https://github.com/bolderflight/eigen/) library and the [BFS Statistics](https://github.com/bolderflight/statistics) library.

### gyro_integration
This demonstrates integrating gyro data at 1 kHz to estimate attitude. Gyro data works well for estimating attitude in dynamic environments, but drifts over time. The gyro is calibrated during startup. Requires the [BFS Invensense IMU](https://github.com/bolderflight/invensense-imu) library, the [BFS Eigen](https://github.com/bolderflight/eigen/) library, the [BFS Statistics](https://github.com/bolderflight/statistics) library, and the [BFS Units](https://github.com/bolderflight/units) library.

### mag_cal
Calibrates the LIS3MDL magnetometer by having the user slowly rotate the board over 60 seconds while trying to have each axis draw a sphere. Requires the [BFS Invensense IMU](https://github.com/bolderflight/invensense-imu) library, the [BFS LIS3MDL](https://github.com/bolderflight/lis3mdl) library and the [BFS Eigen](https://github.com/bolderflight/eigen/) library.

### tilt_compass
Demonstrates a tilt compass to estimate attitude and heading. The gyro is calibrated during startup and the accel and mag calibration factors from *accel_cal* and *mag_cal* are loaded from EEPROM. Requires the [BFS Invensense IMU](https://github.com/bolderflight/invensense-imu) library, the [BFS LIS3MDL](https://github.com/bolderflight/lis3mdl) library, the [BFS Eigen](https://github.com/bolderflight/eigen/) library, the [BFS Statistics](https://github.com/bolderflight/statistics) library, the [BFS Navigation](https://github.com/bolderflight/navigation) library, and the [BFS Units](https://github.com/bolderflight/units) library.

### ahrs
Demonstrates a 7 state EKF to estimate attitude, heading, and real-time gyro biases. The gyro is calibrated during startup and the accel and mag calibration factors from *accel_cal* and *mag_cal* are loaded from EEPROM. Requires the [BFS Invensense IMU](https://github.com/bolderflight/invensense-imu) library, the [BFS LIS3MDL](https://github.com/bolderflight/lis3mdl) library, the [BFS Eigen](https://github.com/bolderflight/eigen/) library, the [BFS Statistics](https://github.com/bolderflight/statistics) library, and the [BFS Units](https://github.com/bolderflight/units) library.

### wireless_air_data
This example demonstrates setting up a wireless air data probe to work with PDAS using the PDAS external data inputs. The code samples data from static and differential pressure transducers (AMS5915-1200B and AMS5915-0020D). The differential pressure bias is removed on startup. Pressures are low pass filtered and used to estimate airspeed (m/s) and pressure altitude (m). Two potentiometers are read over analog, simulating angle of attack and flank angle vanes. Polynomial evaluation is performed to convert the measured voltages to angles. This data is packaged in the PDAS external data format and sent to PDAS over a wireless radio modem. The following libraries are needed:
   * [AMS5915](https://github.com/bolderflight/ams5915)
   * [Statistics](https://github.com/bolderflight/statistics)
   * [Airdata](https://github.com/bolderflight/airdata)
   * [Checksum](https://github.com/bolderflight/checksum/)
   * [Polytools](https://github.com/bolderflight/polytools)
