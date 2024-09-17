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
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32.h>

#include "config.h"
// #include "motor.h"
#include "motor_control.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t twist_subscriber;
rcl_publisher_t checking_output_publisher;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Float32 checking_output_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

enum states 
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

BTS7960 motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_B, MOTOR1_IN_A);
BTS7960 motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_B, MOTOR2_IN_A);
BTS7960 motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_B, MOTOR3_IN_A);
BTS7960 motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_B, MOTOR4_IN_A);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER, 
    ROBOT_DIAMETER
);

Odometry odometry;
IMU imu_sensor;

void setup() 
{
    pinMode(LED_PIN, OUTPUT);

    bool imu_ok = imu_sensor.init();
    if(!imu_ok)
    {
        flashLED(3);
    }
    
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    // motor1_controller.init();
    // motor2_controller.init();
    // motor3_controller.init();
    // motor4_controller.init();
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));
    // create odometry publisher
    RCCHECK(rclc_publisher_init_default( 
        &odom_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"
    ));
    // create IMU publisher
    RCCHECK(rclc_publisher_init_default( 
        &imu_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    ));
    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default( 
        &twist_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));
    // create troubleshooting publisher
    RCCHECK(rclc_publisher_init_default( 
        &checking_output_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "checking_output"
    ));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, & allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &twist_subscriber, 
        &twist_msg, 
        &twistCallback, 
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);

    return true;
}

void loop() {
    switch (state) 
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) 
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) 
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
                publishData();
                moveBase();
            }
            break;
        case AGENT_DISCONNECTED:
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
}

void twistCallback(const void * msgin) 
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    prev_cmd_time = millis();
}


bool destroyEntities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);
    
    return true;
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();
}
float prevT  = 0;
float deltaT = 0;
void moveBase()
{   
    float currT = millis();
    float deltaT = ((float)(currT - prevT)) / 0.001;
    prevT = currT;
    // brake if there's no command received, or when it's only the first command sent
    if(((millis() - prev_cmd_time) >= 200)) 
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        digitalWrite(LED_PIN, HIGH);
    }
    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x, 
        twist_msg.linear.y, 
        twist_msg.angular.z
    );

    // get the current speed of each motor
    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();
    float current_rpm3 = motor3_encoder.getRPM();
    float current_rpm4 = motor4_encoder.getRPM();

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    float controlled_motor1 = motor1_pid.control_speed(req_rpm.motor1/60, motor1_encoder.read(), deltaT);
    float controlled_motor2 = motor2_pid.control_speed(req_rpm.motor2/60, motor2_encoder.read(), deltaT);
    float controlled_motor3 = motor3_pid.control_speed(req_rpm.motor3/60, motor3_encoder.read(), deltaT);
    float controlled_motor4 = motor4_pid.control_speed(req_rpm.motor4/60, motor4_encoder.read(), deltaT);
    if (req_rpm.motor1 == 0){
        controlled_motor1 = 0;
        controlled_motor2 = 0;
        controlled_motor3 = 0;
        controlled_motor4 = 0;
    } 
    motor1_controller.spin(controlled_motor1);
    motor2_controller.spin(controlled_motor2);
    motor3_controller.spin(controlled_motor3);
    motor4_controller.spin(controlled_motor4);
    checking_output_msg.data = controlled_motor1;
    RCSOFTCHECK(rcl_publish(&checking_output_publisher, &checking_output_msg, NULL));

    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rpm1, 
        current_rpm2, 
        current_rpm3, 
        current_rpm4
    );

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt, 
        current_vel.linear_x, 
        current_vel.linear_y, 
        current_vel.angular_z
    );
}

void publishData()
{
    odom_msg = odometry.getData();
    imu_msg = imu_sensor.getData();

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis(); 
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop() 
{
    flashLED(2);
    
}

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}