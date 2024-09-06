// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

#define LED_PIN 13 //used for debugging status

//uncomment the base you're building
// #define LINO_BASE DIFFERENTIAL_DRIVE       // 2WD and Tracked robot w/ 2 motors
// #define LINO_BASE SKID_STEER            // 4WD robot
// #define LINO_BASE Mecanume               // Mecanum drive robot
#define LINO_BASE OMNI

#define USE_BTS7960_MOTOR_DRIVER        // BTS7970 Motor Driver

#define USE_MPU6050_IMU

#define K_P 15               // P constant
#define K_I 5                // I constant
#define K_D 0                // D constant

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

//define your robot' specs here
#define MOTOR_MAX_RPM 550                   // motor's max RPM          
#define MAX_RPM_RATIO 0.85                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
#define MOTOR_OPERATING_VOLTAGE 24          // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 23.5        // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 24     // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 3840                // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 3840                // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 3840                // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 3840                // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.05                 // wheel's diameter in meters
#define ROBOT_DIAMETER 0.1                  // distance between left and right wheels
#define PWM_BITS 10                         // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 20000                 // PWM Frequency

// INVERT ENCODER COUNTS
#define MOTOR1_ENCODER_INV false 
#define MOTOR2_ENCODER_INV false 
#define MOTOR3_ENCODER_INV false 
#define MOTOR4_ENCODER_INV false 

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV false
#define MOTOR4_INV false


// MOTOR PINS 
#ifdef USE_BTS7960_MOTOR_DRIVER
  const int cw[4] = {36,5,22,3};
  const int ccw[4] ={37,6,23,4};
  const int pwm[4] ={-1,-1,-1,-1};
  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

#endif
