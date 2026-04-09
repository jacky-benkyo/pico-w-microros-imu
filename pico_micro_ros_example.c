#include <stdio.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/i2c.h"
#include <sensor_msgs/msg/imu.h>
#include "pico/cyw43_arch.h"

// GY-801 I2C Configuration / I2C 設定
#define I2C_PORT i2c0
#define ADXL345_ADDR 0x53
#define GYRO_ADDR 0x69


// Registers / 寄存器地址
#define ADXL345_REG_DATA 0x32
#define ADXL345_REG_POWER_CTL 0x2D
#define GYRO_CTRL_REG1 0x20
#define GYRO_REG_DATA 0x28

//const uint LED_PIN = 25;

// Scaling Factors / 比例轉換
// Accel: 1 LSB = 3.9mg. Gyro: 1 LSB = 0.00875 dps (at 250dps)
const float ACCEL_SCALE = 0.0039f * 9.80665f;
const float GYRO_SCALE = 0.00875f;
const float ALPHA = 0.98f; // Filter coefficient / 濾波系數

// Global variables /
rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;
sensor_msgs__msg__Imu msg; // 換咗呢個 Message Type
float angle_pitch = 0.0f;
float gyro_offset_y = 0.0f;
absolute_time_t last_time;

//void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
//{
//    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
//    msg.data++;
//}

// Initialize sensors gy801
void init_gy801() {
    i2c_init(I2C_PORT, 400 * 1000); // 400kHz 快啲
    gpio_set_function(4, GPIO_FUNC_I2C); // SDA
    gpio_set_function(5, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(4);
    gpio_pull_up(5);

    // Start ADXL345：寫入 0x08 入去 0x2D register (Measurement Mode)
    uint8_t accel_init[] = {0x2D, 0x08};
    i2c_write_blocking(I2C_PORT, ADXL345_ADDR, accel_init, 2, false);

    // Start L3G4200D：寫入 0x0F (Normal mode, all axes enabled) 入去 0x20
    uint8_t gyro_init[] = {GYRO_CTRL_REG1, 0x0F};
    i2c_write_blocking(i2c0, GYRO_ADDR, gyro_init, 2, false);
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

void read_gyroscope(float *gy_rate) {
uint8_t reg_gyro = GYRO_REG_DATA | 0x80; // MSB must be 1 for multiple byte read
    uint8_t data_gyro[6];
    i2c_write_blocking(I2C_PORT, GYRO_ADDR, &reg_gyro, 1, true);
    i2c_read_blocking(I2C_PORT, GYRO_ADDR, data_gyro, 6, false);

    // We focus on Pitch (Y-axis rotation) / 我哋主要睇 Pitch (Y軸旋轉) , bitwise 運算順序
    int16_t raw_gy = (int16_t)(data_gyro[3] << 8 | data_gyro[2]);
    *gy_rate = raw_gy * GYRO_SCALE;
}

void calibrate_gyro() {
    float sum = 0;
    int samples = 500;
    for (int i = 0; i < samples; i++) {
        float raw_gy;
        read_gyroscope(&raw_gy); // 用返你寫嗰個 function
        sum += raw_gy;
        sleep_ms(2);
    }
    gyro_offset_y = sum / samples; // 呢個就係你粒 Chip 嘅「靜止誤差」
}

/*
void complementary_filter (float *ax, float *ay, float *az) {
    float accel_pitch = atan2f(*ax, sqrtf(*ay * *ay + *az * *az)) * 57.295f;
    angle_pitch = ALPHA * (angle_pitch + gy_rate * dt) + (1.0f - ALPHA) * accel_pitch;
}

*/

// Timer callback: Executed every 50ms / 定時器回調：每 50ms 執行一次
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    absolute_time_t now = get_absolute_time();
    float dt = absolute_time_diff_us(last_time, now) / 1000000.0f;
    last_time = now;

    float ax, ay, az;
    float gy_rate;
    float raw_gy_rate;

    read_accel(&ax, &ay, &az);
    read_gyroscope(&raw_gy_rate);
    
    // Complementary Filter / 互補濾波
    float accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.295f;   
    float gy_rate = raw_gy_rate - gyro_offset_y;
    angle_pitch = ALPHA * (angle_pitch + gy_rate * dt) + (1.0f - ALPHA) * accel_pitch;

    // Convert Euler to Quaternion (Pitch only) / 角度轉四元數 (只計 Pitch)
    float p = angle_pitch * 0.5f * 0.01745329f; // Convert to radians / 轉弧度
    msg.orientation.w = cosf(p);
    msg.orientation.x = 0;
    msg.orientation.y = sinf(p);
    msg.orientation.z = 0;

    // 填入 IMU 數據 (線性加速度)
    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;
    msg.angular_velocity.y = gy_rate * 0.01745329f; // Convert to rad/s

    // Set Frame ID for RViz / 設定 RViz 識別名
    msg.header.frame_id.data = "pico_frame";
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
    last_time = get_absolute_time();
    
    calibrate_gyro();
    
    //initialize the WIFI chipset on PICOW
    if (cyw43_arch_init()) {
        printf("WiFi init failed");
    return -1;
    }

    // Switch on the PICOW LED 
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); 


    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
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
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(50),
        timer_callback);
    
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    
    msg.header.frame_id.capacity = 20; // Allocate memory for string / 為字串分配記憶體
    gpio_put(LED_PIN, 1);

    //msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
