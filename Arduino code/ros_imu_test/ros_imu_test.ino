////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib2-Teensy
//
//  Copyright (c) 2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include "I2Cdev.h"
#include "RTIMULib.h"

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/String.h>

RTIMU *imu;                                           // the IMU object
RTIMUSettings *settings;                              // the settings object

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  100                         // interval between pose displays

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  57600

#define G_2_MPSS 9.80665
#define uT_2_T 1000000

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
char* imu_frame_id_;
sensor_msgs::MagneticField mag_msg;
geometry_msgs::Vector3 euler_msg;

ros::Publisher imu_pub_("imu_data", &imu_msg);
ros::Publisher magnetometer_pub_("mag_data", &mag_msg);
ros::Publisher euler_pub_("euler_data", &euler_msg);

void setup()
{
    nh.initNode();
    nh.advertise(imu_pub_);
    nh.advertise(magnetometer_pub_);
    nh.advertise(euler_pub_);
    
    int errcode;
  
    Serial.begin(SERIAL_PORT_SPEED);
//    while (!Serial) {
//        ; // wait for serial port to connect. 
//    }
    Wire.begin();
    settings = new RTIMUSettings();
    imu = RTIMU::createIMU(settings);                        // create the imu object

   
    Serial.print("TeensyIMU starting using device "); Serial.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
      Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
  
    if (imu->getCompassCalibrationValid())
        Serial.println("Using compass calibration");
    else
        Serial.println("No valid compass calibration data");
        
    // set up any fusion parameters here
    
    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    lastDisplay = lastRate = millis();
    sampleCount = 0;
}

void loop()
{  
    unsigned long now = millis();
    unsigned long delta;
    RTIMU_DATA imuData;
  
    if (imu->IMURead()) {                                // get the latest data if ready yet
        imuData = imu->getIMUData();
        sampleCount++;
        if ((delta = now - lastRate) >= 5000) {          //48 readings per second
            Serial.print("Sample rate: "); Serial.print(sampleCount);
            if (imu->IMUGyroBiasValid())
                Serial.print(", gyro bias valid");
            else
                Serial.print(", calculating gyro bias");
        
            if (!imu->getCompassCalibrationValid()) {
                if (imu->getRuntimeCompassCalibrationValid())
                    Serial.print(", runtime mag cal valid");
                else     
                    Serial.print(", runtime mag cal not valid");
            }
            Serial.println();
            sampleCount = 0;
            lastRate = now;
        }
        if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
            lastDisplay = now;
//            Serial.print(RTMath::displayRadians("Gyro:", imuData.gyro));       // gyro data
//            Serial.print(RTMath::displayRadians("Accel:", imuData.accel));    // accel data
//            Serial.print(RTMath::displayRadians("Mag:", imuData.compass));     // compass data
//            Serial.print(RTMath::displayDegrees("Pose:", imuData.fusionPose)); // fused output
//            Serial.println();

          imu_msg.header.stamp = nh.now();
          imu_msg.header.frame_id = imu_frame_id_;
          imu_msg.orientation.x = imuData.fusionQPose.x();
          imu_msg.orientation.y = imuData.fusionQPose.y();
          imu_msg.orientation.z = imuData.fusionQPose.z();
          imu_msg.orientation.w = imuData.fusionQPose.scalar();
      
          imu_msg.angular_velocity.x = imuData.gyro.x();
          imu_msg.angular_velocity.y = imuData.gyro.y();
          imu_msg.angular_velocity.z = imuData.gyro.z();
      
          imu_msg.linear_acceleration.x = imuData.accel.x(); //* G_2_MPSS;
          imu_msg.linear_acceleration.y = imuData.accel.y() * G_2_MPSS;
          imu_msg.linear_acceleration.z = imuData.accel.z() * G_2_MPSS;
      
          imu_pub_.publish(&imu_msg);
      
          //if (imuData.compassValid)
    
          mag_msg.header.frame_id=imu_frame_id_;
          mag_msg.header.stamp=nh.now();
    
          mag_msg.magnetic_field.x = imuData.compass.x()/uT_2_T;
          mag_msg.magnetic_field.y = imuData.compass.y()/uT_2_T;
          mag_msg.magnetic_field.z = imuData.compass.z()/uT_2_T;
    
          magnetometer_pub_.publish(&mag_msg);
    
          //if (euler_pub_ != NULL)
          euler_msg.x = imuData.fusionPose.x();
          euler_msg.y = imuData.fusionPose.y();
          euler_msg.z = -imuData.fusionPose.z();
          //euler_msg.z = (-imuData.fusionPose.z()) - declination_radians_;
          euler_pub_.publish(&euler_msg);
            
        }

    
    
    nh.spinOnce();
    }
}

