# Self-Balancing Robot

A compact, 3D-printed self-balancing robot powered by an ESP32 and an MPU-6500 IMU. The robot uses a PID (Proportional-Integral-Derivative) controller to continuously measure its angle and adjust two DC motors to keep itself balanced.

This project is a great introduction to robotics, embedded systems, and control theory.

## Features

- 🤖 ESP32-based controller
- ⚖️ PID balancing algorithm
- 📈 MPU-6500 IMU (compatible with MPU-6050/9250)
- 🖨️ Fully 3D printable frame
- 🔧 Easy to modify and customize
- 📚 Great project for learning control systems

---

## Hardware

### Electronics

- ESP32 Dev Board
- L298N Motor Driver
- MPU-6500 IMU
  - MPU-6050 and MPU-9250 are also compatible
- 2× DC Gear Motors
- 2S LiPo Battery
- Jumper Wires
- Power Switch (optional)

---

## 3D Printed Parts

The printable STL files are available here:

**https://www.printables.com/model/1767117-esp32-pid-self-balancing-robot**

Print:
- 2× Side Panels
- 2× Motor Brackets
- 1× of every other part

---

## Software

Upload the Arduino code to the ESP32 using the Arduino IDE.

Required libraries include:
- Wire
- MPU6050 (or compatible MPU library)

---

## Assembly

1. Print all required parts.
2. Assemble the chassis.
3. Install the motors.
4. Mount the ESP32, motor driver, and MPU.
5. Wire everything together.
6. Upload the code.
7. Tune the PID values for your specific robot.

---

## PID Tuning

Every robot is slightly different, so the PID values will likely need adjustment depending on:
- Motor speed
- Weight
- Battery voltage
- Wheel size
- Center of gravity

Start with small values and gradually tune until the robot balances smoothly.

---

## License

This project is licensed under the **CERN Open Hardware License v2** (or replace with your preferred license).

---

## Contributing

Feel free to fork the project, improve the design, or submit pull requests.

---

## Credits

Designed and programmed by **Chahatesh**.

If you build one, I'd love to see it!
