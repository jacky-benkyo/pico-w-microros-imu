// Copyright (c) 2026 jacky-benkyo. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include <stdio.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <time.h>

//#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/i2c.h"
#include <sensor_msgs/msg/imu.h>
#include "pico/cyw43_arch.h"

// GY-801 I2C Configuration 
#define I2C_PORT i2c0
#define ADXL345_ADDR 0x53
#define GYRO_ADDR 0x69


// Registers 
#define ADXL345_REG_DATA 0x32
#define ADXL345_REG_POWER_CTL 0x2D
#define GYRO_CTRL_REG1 0x20
#define GYRO_REG_DATA 0x28

//const uint LED_PIN = 25;

// Scaling Factors 
// Accel: 1 LSB = 3.9mg. Gyro: 1 LSB = 0.00875 dps (at 250dps)
const float ACCEL_SCALE = 0.0039f * 9.80665f;
const float GYRO_SCALE = 0.00875f;
const float ALPHA = 0.98f; // Filter coefficient 

// Global variables /
rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;
sensor_msgs__msg__Imu msg; // IMU Message Type
float angle_pitch = 0.0f;
float gyro_offset_x = 0.0f;
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;
absolute_time_t last_time;

//void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
//{
//    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
//    msg.data++;
//}

// Initialize sensors gy801
void init_gy801() {
    i2c_init(I2C_PORT, 400 * 1000); // 400kHz 
    gpio_set_function(4, GPIO_FUNC_I2C); // SDA
    gpio_set_function(5, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(4);
    gpio_pull_up(5);

    // Start ADXL345： Write 0x08 入去 0x2D register (Measurement Mode)
    uint8_t accel_init[] = {0x2D, 0x08};
    i2c_write_blocking(I2C_PORT, ADXL345_ADDR, accel_init, 2, false);

    // Start L3G4200D: Write 0x0F (Normal mode, all axes enabled) into 0x20
    uint8_t gyro_init[] = {GYRO_CTRL_REG1, 0x0F};
    i2c_write_blocking(i2c0, GYRO_ADDR, gyro_init, 2, false);
}

void read_accel(float *ax, float *ay, float *az) {
    uint8_t reg = ADXL345_REG_DATA;
    uint8_t data[6]; // 6 bytes (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    i2c_write_blocking(I2C_PORT, ADXL345_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ADXL345_ADDR, data, 6, false);

    int16_t raw_x = (data[1] << 8) | data[0];
    int16_t raw_y = (data[3] << 8) | data[2];
    int16_t raw_z = (data[5] << 8) | data[4];

    *ax = raw_x * ACCEL_SCALE;
    *ay = raw_y * ACCEL_SCALE;
    *az = raw_z * ACCEL_SCALE;
}

void read_gyroscope(float *gx, float *gy, float *gz){
    uint8_t reg_gyro = GYRO_REG_DATA | 0x80; // MSB must be 1 for multiple byte read, auto-increment
    uint8_t data_gyro[6];
    
    i2c_write_blocking(I2C_PORT, GYRO_ADDR, &reg_gyro, 1, true);
    i2c_read_blocking(I2C_PORT, GYRO_ADDR, data_gyro, 6, false);

    // We focus on Pitch (Y-axis rotation) / Pitch Y , bitwise 
    int16_t raw_gx = (int16_t)(data_gyro[1] << 8 | data_gyro[0]);
    int16_t raw_gy = (int16_t)(data_gyro[3] << 8 | data_gyro[2]);
    int16_t raw_gz = (int16_t)(data_gyro[5] << 8 | data_gyro[4]);

    // Convert into degree per second (dps)
    *gx = raw_gx * GYRO_SCALE;   
    *gy = raw_gy * GYRO_SCALE;
    *gz = raw_gz * GYRO_SCALE;    
}

void calibrate_gyro() {
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int samples = 500;
    for (int i = 0; i < samples; i++) {
        float gx, gy, gz;
        read_gyroscope(&gx, &gy, &gz);
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        sleep_ms(2);
    }
    gyro_offset_x = sum_x / samples;
    gyro_offset_y = sum_y / samples;
    gyro_offset_z = sum_z / samples;

    printf("Calibration Done. Offset Y: %.4f\n", gyro_offset_y);
}

/*
void complementary_filter (float *ax, float *ay, float *az) {
    float accel_pitch = atan2f(*ax, sqrtf(*ay * *ay + *az * *az)) * 57.295f;
    angle_pitch = ALPHA * (angle_pitch + gy_rate * dt) + (1.0f - ALPHA) * accel_pitch;
}

*/

// Timer callback: Executed every 50ms 
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    //syncronize the timestamp with agent
    int64_t nanoseconds = rmw_uros_epoch_nanos();
    msg.header.stamp.sec = nanoseconds / 1000000000;
    msg.header.stamp.nanosec = nanoseconds % 1000000000;
    
    //calculate dt
    absolute_time_t now = get_absolute_time();
    float dt = absolute_time_diff_us(last_time, now) / 1000000.0f;
    last_time = now;
    
    //read sensor data
    float ax, ay, az;
    float gx, gy, gz;
    read_accel(&ax, &ay, &az);
    read_gyroscope(&gx, &gy, &gz);
    
    // complementary Filter 
    float accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.295f;   
    float gx_rate = gx - gyro_offset_x;
    float gy_rate = gy - gyro_offset_y;
    float gz_rate = gz - gyro_offset_z;
    angle_pitch = ALPHA * (angle_pitch + gy_rate * dt) + (1.0f - ALPHA) * accel_pitch;

    // Convert Euler to Quaternion (Pitch only) 
    float p = angle_pitch * 0.5f * 0.01745329f; // Convert to radians 
    msg.orientation.w = cosf(p);
    msg.orientation.x = 0;
    msg.orientation.y = sinf(p);
    msg.orientation.z = 0;

    
    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;
    
    // 填入 IMU Message (ROS 2 Standard: rad/s)
    msg.angular_velocity.x = gx * 0.01745329f;
    msg.angular_velocity.y = gy_rate * 0.01745329f; // for balancing
    msg.angular_velocity.z = gz * 0.01745329f;

    // Switch on the PICOW LED 
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); 

    // Set Frame ID for RViz 
    msg.header.frame_id.data = "pico_frame";
    // Handle Orientation and Angular Velocity later , set 0 
    // Check Publish   
    rcl_ret_t pub_ret = rcl_publish(&publisher, &msg, NULL);
    
}

/*
// 2. Read x accel in int16
int16_t read_accel_x() {
    uint8_t reg = ADXL345_REG_DATA;
    uint8_t data[2];
    
    i2c_write_blocking(I2C_PORT, ADXL345_ADDR, &reg, 1, true);
    //Read 2  bytes (LSB 同 MSB)
    i2c_read_blocking(I2C_PORT, ADXL345_ADDR, data, 2, false);
    return (int16_t)((data[1] << 8) | data[0]);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    // Write x accel into msg
    msg.data = (int32_t)read_accel_x();
    rcl_publish(&publisher, &msg, NULL);
}
*/

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    init_gy801(); 
    last_time = get_absolute_time();
    
    calibrate_gyro();
    
    //initialize the WIFI chipset on PICOW
    if (cyw43_arch_init()) {
        printf("WiFi init failed");
    return -1;
    }

  
    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
 
    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }
    else
    {
        rmw_uros_sync_session(1000);
    }
   
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node; 
    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        //ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
       "pico_imu"
       //"pico_publisher"
    );

    rcl_timer_t timer;
    rclc_timer_init_default2(
    &timer,
    &support,
    RCL_MS_TO_NS(50),
    timer_callback,
    true); // true -- autostart
    
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    

    // Allocate memory for string 
    char frame_id_buffer[20];
    msg.header.frame_id.data = frame_id_buffer;
    msg.header.frame_id.capacity = 20;
    snprintf(msg.header.frame_id.data, 20, "pico_frame");
    msg.header.frame_id.size = strlen(msg.header.frame_id.data);

    //msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
