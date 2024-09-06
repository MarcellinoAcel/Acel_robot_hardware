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

#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include <Encoder.h>
#include "kinematics.h"

#define SAMPLE_TIME 10 //s

Encoder enc1(14,15);
Encoder enc2(11,12);
Encoder enc3(17,16);
Encoder enc4(9,10);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, pwm[0], cw[0], ccw[0]);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, pwm[1], cw[1], ccw[1]);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, pwm[2], cw[2], ccw[2]);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, pwm[3], cw[3], ccw[3]);

Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER, 
    ROBOT_DIAMETER
);

long long int counts_per_rev[4];
int total_motors = 4;
Motor *motors[4] = {&motor1_controller, &motor2_controller, &motor3_controller, &motor4_controller};
Encoder *encoders[4] = {&enc1, &enc2, &enc3, &enc4};
String labels[4] = {"FRONT LEFT - M1: ", "FRONT RIGHT - M2: ", "REAR LEFT - M3: ", "REAR RIGHT - M4: "};

void setup()
{
    Serial.begin(9600);
    while (!Serial) {
    }
    Serial.println("Sampling process will spin the motors at its maximum RPM.");
    Serial.println("Please ensure that the robot is ELEVATED and there are NO OBSTRUCTIONS to the wheels.");
    Serial.println("");
    Serial.println("Type 'spin' and press enter to spin the motors.");
    Serial.println("Type 'sample' and press enter to spin the motors with motor summary.");
    Serial.println("Press enter to clear command.");
    Serial.println("");
}

void loop()
{
    static String cmd = "";

    while (Serial.available())
    {
        char character = Serial.read(); 
        cmd.concat(character); 
        Serial.print(character);
        delay(1);
        if(character == '\r' and cmd.equals("spin\r"))
        {
            cmd = "";
            Serial.println("\r\n");
            sampleMotors(0);
        }
        else if(character == '\r' and cmd.equals("sample\r"))
        {
            cmd = "";
            Serial.println("\r\n");
            sampleMotors(1);
        }
        else if(character == '\r')
        {
            Serial.println("");
            cmd = "";
        }
    }
}

void sampleMotors(bool show_summary)
{
    if(Kinematics::LINO_BASE == Kinematics::DIFFERENTIAL_DRIVE)
    {
        total_motors = 2;
    }

    float measured_voltage = constrain(MOTOR_POWER_MEASURED_VOLTAGE, 0, MOTOR_OPERATING_VOLTAGE);
    float scaled_max_rpm = ((measured_voltage / MOTOR_OPERATING_VOLTAGE) * MOTOR_MAX_RPM);
    float total_rev = scaled_max_rpm * (SAMPLE_TIME / 60.0);

    for(int i=0; i<total_motors; i++)
    {
        Serial.print("SPINNING ");
        Serial.print(labels[i]);

        unsigned long start_time = micros();
        unsigned long last_status = micros();

        encoders[i]->write(0);
        while(true)
        {
            if(micros() - start_time >= SAMPLE_TIME * 1000000)
            {
                motors[i]->spin(0);
                Serial.println("");
                break;
            }

            if(micros() - last_status >= 1000000)
            {
                last_status = micros();
                Serial.print(".");
            }

            motors[i]->spin(PWM_MAX);
        }
        
        counts_per_rev[i] = encoders[i]->read() / total_rev;
    }
    if(show_summary)
        printSummary();
}

void printSummary()
{
    Serial.println("\r\n================MOTOR ENCODER READINGS================");
    Serial.print(labels[0]);
    Serial.print(encoders[0]->read());
    Serial.print(" ");

    Serial.print(labels[1]);
    Serial.println(encoders[1]->read());

    Serial.print(labels[2]);
    Serial.print(encoders[2]->read());
    Serial.print(" ");

    Serial.print(labels[3]);
    Serial.println(encoders[3]->read());
    Serial.println("");

    Serial.println("================COUNTS PER REVOLUTION=================");
    Serial.print(labels[0]);
    Serial.print(counts_per_rev[0]);
    Serial.print(" ");

    Serial.print(labels[1]);
    Serial.println(counts_per_rev[1]);
    
    Serial.print(labels[2]);
    Serial.print(counts_per_rev[2]);
    Serial.print(" ");

    Serial.print(labels[3]);
    Serial.println(counts_per_rev[3]);
    Serial.println("");

    Serial.println("====================MAX VELOCITIES====================");
    float max_rpm = kinematics.getMaxRPM();
    
    Kinematics::velocities max_linear = kinematics.getVelocities(max_rpm, max_rpm, max_rpm, max_rpm);
    Kinematics::velocities max_angular = kinematics.getVelocities(-max_rpm, max_rpm,-max_rpm, max_rpm);

    Serial.print("Linear Velocity: +- ");
    Serial.print(max_linear.linear_x);
    Serial.println(" m/s");

    Serial.print("Angular Velocity: +- ");
    Serial.print(max_angular.angular_z);
    Serial.println(" rad/s");
}