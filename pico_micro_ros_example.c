#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
//#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/i2c.h"
#include <sensor_msgs/msg/imu.h>

// GY-801 I2C 地址
#define I2C_PORT i2c0
#define ADXL345_ADDR 0x53
#define ADXL345_REG_DATA 0x32
#define ADXL345_REG_POWER_CTL 0x2D


const uint LED_PIN = 25;
// 物理轉換參數：ADXL345 在 +-2g 模式下，1 LSB = 3.9mg
const float ACCEL_SCALE = 0.0039 * 9.80665;

rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;
sensor_msgs__msg__Imu msg; // 換咗呢個 Message Type

//void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
//{
//    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
//    msg.data++;
//}

void init_gy801() {
    i2c_init(I2C_PORT, 400 * 1000); // 400kHz 快啲
    gpio_set_function(4, GPIO_FUNC_I2C); // SDA
    gpio_set_function(5, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(4);
    gpio_pull_up(5);

    // 啟動 ADXL345：寫入 0x08 入去 0x2D register (Measurement Mode)
    uint8_t buf[] = {ADXL345_REG_POWER_CTL, 0x08};
    i2c_write_blocking(I2C_PORT, ADXL345_ADDR, buf, 2, false);
}

void read_accel(float *ax, float *ay, float *az) {
    uint8_t reg = ADXL345_REG_DATA;
    uint8_t data[6]; // 一口氣讀 6 bytes (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    i2c_write_blocking(I2C_PORT, ADXL345_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ADXL345_ADDR, data, 6, false);

    int16_t raw_x = (data[1] << 8) | data[0];
    int16_t raw_y = (data[3] << 8) | data[2];
    int16_t raw_z = (data[5] << 8) | data[4];

    *ax = raw_x * ACCEL_SCALE;
    *ay = raw_y * ACCEL_SCALE;
    *az = raw_z * ACCEL_SCALE;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    float ax, ay, az;
    read_accel(&ax, &ay, &az);

    // 填入 IMU 數據 (線性加速度)
    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;
    
    // 平衡機械人暫時唔需要 Orientation 同 Angular Velocity，可以放 0 或者唔填
    
    rcl_publish(&publisher, &msg, NULL);
}

/*
// 2. 讀取 X 軸加速度 in int16
int16_t read_accel_x() {
    uint8_t reg = ADXL345_REG_DATA;
    uint8_t data[2];
    // 先話畀佢聽想讀邊個位
    i2c_write_blocking(I2C_PORT, ADXL345_ADDR, &reg, 1, true);
    // 讀返 2 粒 bytes 返嚟 (LSB 同 MSB)
    i2c_read_blocking(I2C_PORT, ADXL345_ADDR, data, 2, false);
    return (int16_t)((data[1] << 8) | data[0]);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    // 將讀到嘅 X 軸數據擺入 msg
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

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        //ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
       "pico_imu"
       //"pico_publisher"
    );

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(50),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);

    //msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
