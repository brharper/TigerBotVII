# TigerBotVII
Repository for RIT's TigerBot Senior Design Project

To learn more about how various subsystems are implemented:

In Arduino code folder:

Look at ros_imu_test.ino for how Teensy uses RTIMUlib to read IMU and publishes data to ROS

Look at ros_accel_motor_test.ino for how Teensy controls a motor's speed with messages from ROS (ros_accel_motor is something else)

Look at ros_comm_test_multiple_param.ino, ros_comm_test_v2.ino, and ros_heartbeat_test.ino for code of various communication tests

Look at ros_heartbeat.ino for Teensy side implementation of Heartbeat function


In src/tigerbot7:

Look in launch/ to see roslaunch files, mostly to start up 8 teensy nodes at once.

Look in src/ for various nodes.

-comm_test_multiple and comm_test_multiple v2 are the latest versions of comm test, used in conjunction with ros_comm_test_multiple_param
 and ros_comm_test_v2 respectively

-heartbeat.cpp is ODroid side of heartbeat function, use with ros_heartbeat.ino

-accel_motor.cpp is a node for translating IMU data to an appropriate motor speed. Use in conjuction with ros_imu_test and
 ros_accel_motor_test.ino
