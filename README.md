# Brief-Flight-Computer
Flight computer code for Brief Rocket Project

accel_pyro.ino
  - Sensors: ADXL375 High-G accelerometer and MS5611 high precision barometer
  - Saves sensor data into micro SD card.
  - Barometric apogee detection triggers voltage in selected pyro channel

baro_servo.ino
  - Sensors: BME280. Data saved into micro SD card.
  - Breakwire detects launch and timer triggers servo to actuate.

deploy_only.ino
  - Sensors: none
  - Breakwire detects launch and timer triggers servo to actuate.
  - Used when launching with low battery current/backup system

Three files can be combined easily to customize your avionics software according to your configuration. For example, accel_pyro.ino and baro_servo.ino can be combined to add ADXL375 logging functionality to servo-actuated deployment system. 

 
