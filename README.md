This repository provides a lightweight micro-ROS implementation for the Raspberry Pi Pico W using the GY-801 IMU module (ADXL345 accelerometer and L3G4200D gyroscope). It publishes stabilized IMU data to the ROS 2 environment.
🌟 Key Features

    Pico W LED Support: Fixed the common issue where the onboard LED doesn't light up in micro-ROS environments. This project uses pico_cyw43_arch to properly control the LED via the WiFi chipset.

    Sensor Fusion: Implemented a Complementary Filter (α=0.98) to combine accelerometer and gyroscope data, providing a stable orientation (Pitch) while eliminating gyro drift.

    Auto-Calibration: The system performs an automatic gyroscope offset calibration during the first second of boot-up to ensure accurate "zero" readings.

    RViz2 Integration: Outputs standard sensor_msgs/msg/Imu messages with Quaternion orientation, making it compatible with RViz2 out-of-the-box.

🛠 Hardware Wiring

Connect your GY-801 to the Pico W as follows:
GY-801 Pin	Pico W Pin	Function
VCC	3.3V (Pin 36)	Power
GND	GND (Pin 38)	Ground
SDA	GP4 (Pin 6)	I2C0 SDA
SCL	GP5 (Pin 7)	I2C0 SCL
🚀 Getting Started
1. Build and Flash

Ensure you have the pico-micro-ros-sdk set up in your environment.
Bash

mkdir build && cd build
cmake ..
make

Drag and drop the generated .uf2 file into your Pico W.
2. Run micro-ROS Agent

Connect the Pico W to your PC via USB and run the agent:
Bash

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

Note: The onboard LED will turn ON once a successful connection with the agent is established.
3. Monitor Data

Check the raw IMU data in a new terminal:
Bash

ros2 topic echo /pico_imu

📊 Visualization in RViz2

    Launch rviz2.

    In Global Options, set the Fixed Frame to pico_frame.

    Click Add -> By Topic and select /pico_imu.

    Under the Imu display settings, enable Axes or Box to see the 3D orientation tracking in real-time.

🔧 Technical Specifications

    I2C Frequency: 400kHz (Fast Mode).

    Update Rate: 50ms (20Hz).

    Filter Coefficient: 0.98 (High-pass Gyro + Low-pass Accel).

    Frame ID: pico_frame.

📜 License

This project is licensed under the Apache License 2.0.