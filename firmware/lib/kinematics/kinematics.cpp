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

#include "Arduino.h"
#include "kinematics.h"
#include "math.h"

float toRad(float deg){
    return deg * M_PI/180;
}

Kinematics::Kinematics(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                       float motor_operating_voltage, float motor_power_max_voltage,
                       float wheel_diameter, float wheels_y_distance):
    base_platform_(robot_base),
    wheels_y_distance_(wheels_y_distance),
    wheel_circumference_(PI * wheel_diameter),
    total_wheels_(getTotalWheels(robot_base))
{    
    motor_power_max_voltage = constrain(motor_power_max_voltage, 0, motor_operating_voltage);
    max_rpm_ =  ((motor_power_max_voltage / motor_operating_voltage) * motor_max_rpm) * max_rpm_ratio;
}

Kinematics::rpm Kinematics::calculateRPM(float linear_x, float linear_y, float angular_z)
{

    // float tangential_vel = angular_z * (robot_circumference_);
    float tangential_vel = angular_z;

    //convert m/s to m/min
    float linear_vel_x_mins = linear_x ;
    float linear_vel_y_mins = linear_y ;
    //convert rad/s to rad/min
    float tangential_vel_mins = tangential_vel ;

    float x_rpm = linear_vel_x_mins ;
    float y_rpm = linear_vel_y_mins ;
    float tan_rpm = tangential_vel_mins ;
    
    Kinematics::rpm rpm;

    //calculate for the target motor RPM and direction
    //front-left motor
    float rpm_motor1 = -sin(toRad(45 )) * x_rpm + cos(toRad(45 )) * y_rpm + robot_circumference_ * tan_rpm;
    rpm.motor1 = fmax(-max_rpm_,fmin(rpm_motor1,max_rpm_));

    //front-right motor
    float rpm_motor2 = -sin(toRad(135)) * x_rpm + cos(toRad(135)) * y_rpm + robot_circumference_ * tan_rpm;
    rpm.motor2 = fmax(-max_rpm_,fmin(rpm_motor2,max_rpm_));

    //rear-left motor
    float rpm_motor3 = -sin(toRad(225)) * x_rpm + cos(toRad(225)) * y_rpm + robot_circumference_ * tan_rpm;
    rpm.motor3 = fmax(-max_rpm_,fmin(rpm_motor3,max_rpm_));

    //rear-right motor
    float rpm_motor4 = -sin(toRad(315)) * x_rpm + cos(toRad(315)) * y_rpm + robot_circumference_ * tan_rpm;
    rpm.motor4 = fmax(-max_rpm_,fmin(rpm_motor4,max_rpm_));

    return rpm;
}

Kinematics::rpm Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
    return calculateRPM(linear_x, linear_y, angular_z);
}

Kinematics::velocities Kinematics::getVelocities(float rpm1, float rpm2, float rpm3, float rpm4)
{
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    //convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(-sin(toRad(45)) * rpm1 + -sin(toRad(135)) * rpm2 + -sin(toRad(225)) * rpm3 + -sin(toRad(315)) * rpm4) / total_wheels_); // RPM
    vel.linear_x = average_rps_x * wheel_circumference_; // m/s

    //convert average revolutions per minute in y axis to revolutions per second
    average_rps_y = ((float)(cos(toRad(45)) * rpm1 + cos(toRad(135)) * rpm2 + cos(toRad(225)) * rpm3 + cos(toRad(315)) * rpm4) / total_wheels_); // RPM
    vel.linear_y = average_rps_y * wheel_circumference_; // m/s

    //convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(-rpm1 + rpm2 - rpm3 + rpm4) / total_wheels_);
    vel.angular_z =  (average_rps_a * wheel_circumference_) / (robot_circumference_); //  rad/s

    return vel;
}

int Kinematics::getTotalWheels(base robot_base)
{
    switch(robot_base)
    {
        case DIFFERENTIAL_DRIVE:    return 2;
        case SKID_STEER:            return 4;
        case MECANUM:               return 4;
        case OMNI:                  return 4;
        default:                    return 2;
    }
}

float Kinematics::getMaxRPM()
{
    return max_rpm_;
}