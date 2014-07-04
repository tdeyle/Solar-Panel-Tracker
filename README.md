Solar-Panel-Tracker
===================

Software that enables a solar panel to track the sun with the use of an IMU and almanac based directions.

This will be the repo for any firmware that is used:
- On the Atmel ATmega328P for the IMU sensor pickup and algorithm
- On the Parallax Propeller, 
    - For time keeping,
    - almanac data collection,
    - actuator control,
    - safety controls,
    - serial communication with the ATmega,
    - system debugging and gui

