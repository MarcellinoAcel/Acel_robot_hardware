#include <Arduino.h>
#include <stdio.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32_multi_array.h>
#include "odometry.h"
#include "imu.h"

#include "config.h"
#include "motor_control.h"
#include "kinematics.h"
#include "pid.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
// #include "encoder.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const int motorPWM = 6;
const int motorCW = 7;
const int motorCCW = 8;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);

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
rcl_publisher_t checking_output_motor;
rcl_publisher_t checking_input_motor;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Float32MultiArray checking_output_msg;
std_msgs__msg__Float32MultiArray checking_input_msg;

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

const int enca[4] = { MOTOR1_ENCODER_A, MOTOR2_ENCODER_A, MOTOR3_ENCODER_A, MOTOR4_ENCODER_A};
const int encb[4] = { MOTOR1_ENCODER_B, MOTOR2_ENCODER_B, MOTOR3_ENCODER_B, MOTOR4_ENCODER_B};

volatile long posi[4] = { 0, 0, 0, 0 };

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPS, 
    MAX_RPS_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER, 
    ROBOT_DIAMETER
);

Odometry odometry;
IMU imu_sensor;


bool createEntities(){
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
        &checking_output_motor, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "checking_output"
    ));
    checking_output_msg.data.data = (float*)malloc(4 * sizeof(float)); // Sesuaikan jumlah elemen
    checking_output_msg.data.size = 4;
    RCCHECK(rclc_publisher_init_default( 
        &checking_input_motor, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "checking_input"
    ));
    checking_input_msg.data.data = (float*)malloc(4 * sizeof(float)); // Sesuaikan jumlah elemen
    checking_input_msg.data.size = 4;
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
    // createEntities();
    for (int i = 0; i < 4; i++) {
        pinMode(cw[i], OUTPUT);
        pinMode(ccw[i], OUTPUT);
        analogWriteFrequency(cw[i], PWM_FREQUENCY);
        analogWriteFrequency(ccw[i], PWM_FREQUENCY);
        analogWriteResolution(PWM_BITS);
        analogWrite(cw[i], 0);
        analogWrite(ccw[i], 0);
        pinMode(enca[i], INPUT);
        pinMode(encb[i], INPUT);
    }
    attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
    attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
}
template<int j>
void readEncoder() {
  int b = digitalRead(encb[j]);
  if (b > 0) {
    posi[j]++;
  } else {
    posi[j]--;
  }
}
void setMotor(int cwPin, int ccwPin, float pwmVal) {
    if (pwmVal > 0) {
        analogWrite(cwPin, fabs(pwmVal));
        analogWrite(ccwPin, 0);
    } else if (pwmVal < 0) {
        analogWrite(cwPin, 0);
        analogWrite(ccwPin, fabs(pwmVal));
    } else {
        analogWrite(cwPin, 0);
        analogWrite(ccwPin, 0);
    }
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
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
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
    
    setMotor(cw[0],ccw[0],0);
    setMotor(cw[1],ccw[1],0);
    setMotor(cw[2],ccw[2],0);
    setMotor(cw[3],ccw[3],0);
}
float prevT  = 0;
float deltaT = 0;
void moveBase(){
    sensors_event_t angVelocityData;
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    float currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;
    prevT = currT;
    // brake if there's no command received, or when it's only the first command sent
    if(((millis() - prev_cmd_time) >= 200)) 
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        digitalWrite(LED_PIN, HIGH);
    }
    // get the required rps for each motor based on required velocities, and base used
    Kinematics::rps req_rps;
    req_rps = kinematics.getRPS(
        twist_msg.linear.x, 
        twist_msg.linear.y, 
        twist_msg.angular.z
    ); 

    // get the current speed of each motor
    float current_rps1 = motor1_pid.getPureVal();
    float current_rps2 = motor2_pid.getPureVal();
    float current_rps3 = motor3_pid.getPureVal();
    float current_rps4 = motor4_pid.getPureVal();

    // the required rps is capped at -/+ MAX_rps to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required rps vs measured rps
    float controlled_motor1 = motor1_pid.control_speed(req_rps.motor1, posi[0], deltaT);
    float controlled_motor2 = motor2_pid.control_speed(req_rps.motor2, posi[1], deltaT);
    float controlled_motor3 = motor3_pid.control_speed(req_rps.motor3, posi[2], deltaT);
    float controlled_motor4 = motor4_pid.control_speed(req_rps.motor4, posi[3], deltaT);
    if (req_rps.motor1 == 0 && req_rps.motor2 == 0 && req_rps.motor3 == 0 && req_rps.motor4 == 0 ){
        controlled_motor1 = 0;
        controlled_motor2 = 0;
        controlled_motor3 = 0;
        controlled_motor4 = 0;
    } 
    setMotor(cw[0],ccw[0],controlled_motor4);
    setMotor(cw[1],ccw[1],controlled_motor2);
    setMotor(cw[2],ccw[2],controlled_motor3);
    setMotor(cw[3],ccw[3],controlled_motor1);

    checking_output_msg.data.data[0] = motor1_pid.getFilteredVal();//1
    checking_output_msg.data.data[1] = motor2_pid.getFilteredVal();//2
    checking_output_msg.data.data[2] = motor3_pid.getFilteredVal();//3
    checking_output_msg.data.data[3] = motor4_pid.getFilteredVal();//4

    RCSOFTCHECK(rcl_publish(&checking_input_motor, &checking_input_msg, NULL));

    checking_input_msg.data.data[0] = req_rps.motor1;//1
    checking_input_msg.data.data[1] = req_rps.motor2;//2
    checking_input_msg.data.data[2] = req_rps.motor3;//3
    checking_input_msg.data.data[3] = req_rps.motor4;//4
    
    RCSOFTCHECK(rcl_publish(&checking_output_motor, &checking_output_msg, NULL));

    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rps1, 
        current_rps2, 
        current_rps3, 
        current_rps4
    );

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt, 
        current_vel.linear_x, 
        current_vel.linear_y, 
        angVelocityData.gyro.z
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