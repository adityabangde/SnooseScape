# SnooseScape
SnoosScape is an interactive alarm clock on wheels that ensures you wake up by running away when the alarm goes off!
SnoosScape – The Alarm That Runs Away 🚀
Overview
SnoosScape is an autonomous alarm robot that ensures you wake up by running away when the alarm triggers, requiring you to physically chase it. Built on STM32L476RG with RTOS, it integrates multiple sensors for real-time decision-making.

Key Features
Obstacle Avoidance: VL53L0X sensor array enables real-time path planning.
Human Presence Detection: LD2410C ensures activation only when a person is nearby.
Self-Righting Mechanism: MPU6050 detects flips, and a servo restores balance.
Accurate Timekeeping: DS3231 RTC manages real-time alarms.
User Interaction: SSD1306 OLED displays time, and UART allows setting alarm and actual time.
Precise Motion Control: Rotary slotted encoder provides RPM feedback, and PID control ensures fast response time.
Hardware Components
Microcontroller: STM32L476RG
Sensors: VL53L0X (obstacle avoidance), LD2410C (presence detection), MPU6050 (tilt sensing), Rotary Encoder (RPM feedback)
Motor Control: DC motors with PID-based speed regulation
Clock & Display: DS3231 RTC, SSD1306 OLED
Communication: UART for setting time and alarm
How It Works
DS3231 RTC triggers the alarm.
LD2410C checks for human presence before activation.
Motors move the robot, guided by VL53L0X for obstacle avoidance.
PID-controlled RPM feedback ensures precise speed and response time.
MPU6050 detects flips, and the servo repositions the robot.
UART interface allows users to set time and alarms.
Future Enhancements
🔹 Mobile app integration for remote control.
🔹 Machine learning for adaptive evasion patterns.
🔹 Voice command support.

Contributing
Feel free to fork, create issues, or submit pull requests. Suggestions are always welcome! 🚀
