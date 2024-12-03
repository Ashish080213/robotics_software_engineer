# Module 1 Assignment: Introduction to OOP Concepts in C++ for Robotics

## Tasks

### Task 1: Robot Class Implementation

### How To Run?

- Perform
   ```
   cd ~/assignment_ws/src/robotics_software_engineer/module_1_assignment/build/
   cmake ..
   make
   ```
- Run with this command
   ```
   ./task1
   ```
### OUTPUT-1

   ```
   Robot R1 Initialized
   Robot name is R1
   Robot R1 speed is 2m/s
   Robot R1 weight is 1000g
   Robot R1 size is 30cmx30cmx10cm
   Robot R1 has 3 sensors.

   Robot R2 Initialized
   Robot name is R2
   Robot R2 speed is 3m/s
   Robot R2 weight is 2000g
   Robot R2 size is 40cmx30cmx10cm
   Robot R2 has 6 sensors.

   Robot R1 moving forward by 2m/s.
   Robot R2 moving forward by 3m/s.
   Robot R1 moving backward by 2m/s.
   Robot R2 moving backward by 3m/s.
   Robot R1 stopped.
   Robot R2 stopped.
   ```

### Task 2: Simulating Sensor Readings

### How To Run?

- Perform
   ```
   cd ~/assignment_ws/src/robotics_software_engineer/module_1_assignment/build/
   cmake ..
   make
   ```
- Run with this command
   ```
   ./task2
   ```

### OUTPUT-2

   ```
   Processing Temperature sensor data.....
   Temperature sensor value at 0 sec is 10°C
   Temperature sensor value at 1 sec is 20°C
   Temperature sensor value at 2 sec is 30°C
   Temperature sensor value at 3 sec is 40°C
   Temperature sensor value at 4 sec is 50°C

   Processing Distance sensor data.....
   Distance sensor value at 0 sec is 100cm
   Distance sensor value at 1 sec is 80cm
   Distance sensor value at 2 sec is 60cm
   Distance sensor value at 3 sec is 40cm
   Distance sensor value at 4 sec is 20cm
   ```

### Task 3: Sensor Library Design

### Design a simple sensor library that includes classes for different types of sensors.
### How To Run?

- Perform
   ```
   cd ~/assignment_ws/src/robotics_software_engineer/module_1_assignment/build/
   cmake ..
   make
   ```
- Run with this command
   ```
   ./task3
   ```

### OUTPUT-3

   ```
   Robot created with T1 Temperature Sensor.
   Calibrating Temperature Sensor...
   Processing T1 sensor data.....
   T1 sensor value at 0 sec is 10°C
   T1 sensor value at 1 sec is 20°C
   T1 sensor value at 2 sec is 30°C
   T1 sensor value at 3 sec is 40°C
   T1 sensor value at 4 sec is 50°C
   Applying PID to Sensor values..
   Deciding action based on sensor data...

   Robot created with D1 Distance Sensor.
   Calibrating Distance Sensor...
   Processing D1 Distance sensor data.....
   D1 sensor value at 0 sec is 100cm
   D1 sensor value at 1 sec is 80cm
   D1 sensor value at 2 sec is 60cm
   D1 sensor value at 3 sec is 40cm
   D1 sensor value at 4 sec is 20cm
   Applying PID to Sensor values..
   Deciding action based on sensor data...
   ```

### Create a single-class template.
### How To Run?

- Perform
   ```
   cd ~/assignment_ws/src/robotics_software_engineer/module_1_assignment/build/
   cmake ..
   make
   ```
- Run with this command
   ```
   ./task3b
   ```

### OUTPUT-3b

   ```
   Robot created with T1 Sensor.
   Calibrating T1 Sensor...
   Processing T1 sensor data...
   T1 sensor value at 0 sec is Cool°C
   T1 sensor value at 1 sec is Normal°C
   T1 sensor value at 2 sec is Hot°C
   Applying PID to Sensor values...
   Deciding action based on sensor data...

   Robot created with D1 Sensor.
   Calibrating D1 Sensor...
   Processing D1 sensor data...
   D1 sensor value at 0 sec is 100cm
   D1 sensor value at 1 sec is 80.2cm
   D1 sensor value at 2 sec is 60.8cm
   Applying PID to Sensor values...
   Deciding action based on sensor data...

   Robot created with M1 Sensor.
   Calibrating M1 Sensor...
   Processing M1 sensor data...
   M1 sensor value at 0 sec is R Direction
   M1 sensor value at 1 sec is L Direction
   M1 sensor value at 2 sec is R Direction
   Applying PID to Sensor values...
   Deciding action based on sensor data...
   ```
----

