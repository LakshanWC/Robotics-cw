# 🤖 Smart Automatic Dustbin Robot

An intelligent, Arduino-powered robot that automates waste management through line following, obstacle avoidance, Bluetooth control, and gesture-based bin operation.

## 🚀 Features

- ✅ **Line Following**: Follows a predefined black line path.
- 🚧 **Obstacle Avoidance**: Detects obstacles on the path and navigates around them to rejoin the line.
- 📱 **Bluetooth Control**: Manually control the robot via a Bluetooth-enabled app.
- ✋ **Hover-Based Detection**:
  - Stops the robot when a hand is hovered above it.
  - Opens the **disposable** or **non-disposable** bin lid based on the hand’s position.
- 🔔 **Full Bin Alert**: Buzzer activates when the bin is full.

## 🧠 Tech Stack

- **Microcontroller**: Arduino Uno
- **Sensors**: IR Sensors, Ultrasonic Sensors
- **Communication**: HC-05 Bluetooth Module
- **Actuators**: Servo Motors, Gear Motors
- **Extras**: Buzzer, Custom Chassis

## 📷 Demo
https://www.linkedin.com/posts/lakshan-wc-b02993344_robotics-arduino-smartdustbin-activity-7343654202574811138-CNHn?utm_source=share&utm_medium=member_desktop&rcm=ACoAAFZJX3IBioLocbta1ZfY8ykNmX4Zq8lv6us


## 🛠 How It Works

1. **Startup**: The robot initializes sensors and waits for movement then start moving automaticaly with the given movement method.
2. **Line Following**: Uses IR sensors to stay on track.
3. **Obstacle Detection**: Ultrasonic sensor detects obstacles; the robot avoids them and finds the line again.
4. **Gesture Control**: Hand proximity triggers different actions (e.g., opening a bin lid).
5. **Full Bin Detection**: If the garbage level crosses a threshold, the buzzer alerts the user.

## 👨‍💻 Contributors

- [Samidu](https://github.com/SamiduSamarasinghe)
- [Lakshan](https://github.com/LakshanWC)
- [Wasath](https://github.com/Shady0101)
- [Rukshan](https://github.com/rukaboy)

---
> This project was built as part of a coursework project to explore intelligent robotics and sensor-based automation. We welcome feedback !



