#include "Arduino.h"
#include "kinematics.h"
float toRad(int deg){
    return deg * M_PI /180;
}
Kinematics::Kinematics(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                       float motor_operating_voltage, float motor_power_max_voltage,
                       float wheel_diameter, float robot_diameter):
    base_platform_(robot_base),
    robot_diameter_(robot_diameter),
    wheel_circumference_(PI * wheel_diameter),
    total_wheels_(getTotalWheels(robot_base))
{    
    robot_radius_ = robot_diameter_ / 2;
    motor_power_max_voltage = constrain(motor_power_max_voltage, 0, motor_operating_voltage);
    max_rpm_ =  ((motor_power_max_voltage / motor_operating_voltage) * motor_max_rpm) * max_rpm_ratio;
}

Kinematics::rpm Kinematics::calculateRPM(float linear_x, float linear_y, float angular_z)
{

    float tangential_vel = angular_z * (robot_diameter_ / 2.0);

        
    float linear_vel_x_mins = linear_x * 60.0;
    float linear_vel_y_mins = linear_y * 60.0;
    //convert rad/s to rad/min
    float tangential_vel_mins = tangential_vel * 60.0;

    float x_rpm = linear_vel_x_mins / wheel_circumference_;
    float y_rpm = linear_vel_y_mins / wheel_circumference_;
    float tan_rpm = tangential_vel_mins / wheel_circumference_;

    float a_x_rpm = fabs(x_rpm);
    float a_y_rpm = fabs(y_rpm);
    float a_tan_rpm = fabs(tan_rpm);

    float xy_sum = a_x_rpm + a_y_rpm;
    float xtan_sum = a_x_rpm + a_tan_rpm;

    if(xy_sum >= max_rpm_ && angular_z == 0)
    {
        float vel_scaler = max_rpm_ / xy_sum;

        x_rpm *= vel_scaler;
        y_rpm *= vel_scaler;
    }
    
    else if(xtan_sum >= max_rpm_ && linear_y == 0)
    {
        float vel_scaler = max_rpm_ / xtan_sum;

        x_rpm *= vel_scaler;
        tan_rpm *= vel_scaler;
    }

    Kinematics::rpm rpm;

    //calculate for the target motor RPM and direction
    //front-left motor
    rpm.motor1 = sin(M_PI_4) * x_rpm + cos(M_PI_4) * y_rpm + tan_rpm * robot_radius_;
    rpm.motor1 = fmax(-max_rpm_, fmin(rpm.motor1, max_rpm_));

    //front-right motor
    rpm.motor2 = sin(3 * M_PI_4) * x_rpm + cos(3 * M_PI_4) * y_rpm + tan_rpm * robot_radius_;
    rpm.motor2 = fmax(-max_rpm_, fmin(rpm.motor1, max_rpm_));

    //rear-left motor
    rpm.motor3 = sin(5 * M_PI_4) * x_rpm + cos(5 * M_PI_4) * y_rpm + tan_rpm * robot_radius_;
    rpm.motor3 = fmax(-max_rpm_, fmin(rpm.motor1, max_rpm_));

    //rear-right motor
    rpm.motor4 = sin(7 * M_PI_4) * x_rpm + cos(7 * M_PI_4) * y_rpm + tan_rpm * robot_radius_;
    rpm.motor4 = fmax(-max_rpm_, fmin(rpm.motor1, max_rpm_));

    return rpm;
}

Kinematics::rpm Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
    if(base_platform_ == DIFFERENTIAL_DRIVE || base_platform_ == SKID_STEER)
    {
        linear_y = 0;
    }

    return calculateRPM(linear_x, linear_y, angular_z);;
}

Kinematics::velocities Kinematics::getVelocities(float rpm1, float rpm2, float rpm3, float rpm4)
{
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    // if(base_platform_ == DIFFERENTIAL_DRIVE)
    // {
    //     rpm3 = 0.0;
    //     rpm4 = 0.0;
    // }
 
    // //convert average revolutions per minute to revolutions per second
    // average_rps_x = ((float)(rpm1 + rpm2 + rpm3 + rpm4) / total_wheels_) / 60.0; // RPM
    // vel.linear_x = average_rps_x * wheel_circumference_; // m/s

    // //convert average revolutions per minute in y axis to revolutions per second
    // average_rps_y = ((float)(-rpm1 + rpm2 + rpm3 - rpm4) / total_wheels_) / 60.0; // RPM
    // if(base_platform_ == MECANUM)
    //     vel.linear_y = average_rps_y * wheel_circumference_; // m/s
    // else
    //     vel.linear_y = 0;

    // //convert average revolutions per minute to revolutions per second
    // average_rps_a = ((float)(-rpm1 + rpm2 - rpm3 + rpm4) / total_wheels_) / 60.0;
    // vel.angular_z =  (average_rps_a * wheel_circumference_) / (robot_diameter_ / 2.0); //  rad/s
    
    //convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(sin(45) * rpm1 + sin(135) * rpm2 + sin(225) * rpm3 + sin(315) * rpm4) / total_wheels_) / 60.0; // RPM
    vel.linear_x = average_rps_x * wheel_circumference_; // m/s

    //convert average revolutions per minute in y axis to revolutions per second
    average_rps_y = ((float)(cos(45) * rpm1 + cos(135) * rpm2 + cos(225) * rpm3 + cos(315) * rpm4) / total_wheels_) / 60.0; // RPM
    
    vel.linear_y = average_rps_y * wheel_circumference_; // m/s

    //convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(rpm1 + rpm2 + rpm3 + rpm4) / (4.0 * total_wheels_)) / 60.0;
    vel.angular_z =  (average_rps_a * wheel_circumference_) / (robot_diameter_ / 2.0); //  rad/s
    // }
    return vel;
}

int Kinematics::getTotalWheels(base robot_base)
{
    switch(robot_base){
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