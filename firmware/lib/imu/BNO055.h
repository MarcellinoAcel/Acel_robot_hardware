#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <geometry_msgs/msg/vector3.h>
#include "imu_interface.h"

class BNO055_IMU : public IMUInterface
{
    Adafruit_BNO055 bno;

public:
    BNO055_IMU() : bno(Adafruit_BNO055(55, 0x28, &Wire2)) {}

    void startSensor()
    {
        bno.begin();
    }

    geometry_msgs__msg__Vector3 readAccelerometer() override
    {
        sensors_event_t accel_event;
        geometry_msgs__msg__Vector3 accel_data;
        bno.getEvent(&accel_event, Adafruit_BNO055::VECTOR_ACCELEROMETER);

        accel_data.x = accel_event.acceleration.x * g_to_accel_; // Convert to m/s²
        accel_data.y = accel_event.acceleration.y * g_to_accel_;
        accel_data.z = accel_event.acceleration.z * g_to_accel_;

        return accel_data;
    }

    geometry_msgs__msg__Vector3 readGyroscope() override
    {
        sensors_event_t gyro_event;
        bno.getEvent(&gyro_event, Adafruit_BNO055::VECTOR_GYROSCOPE);
        geometry_msgs__msg__Vector3 gyro_data;

        gyro_data.x = gyro_event.gyro.x; // No conversion needed, values are in rad/s
        gyro_data.y = gyro_event.gyro.y;
        gyro_data.z = gyro_event.gyro.z;

        return gyro_data;
    }
};