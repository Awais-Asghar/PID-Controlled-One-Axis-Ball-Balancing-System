
# PID-Controlled One-Axis Ball Balancing System

![Project Status](https://img.shields.io/badge/status-Completed-brightgreen.svg)
![Platform](https://img.shields.io/badge/platform-Arduino-blue.svg)
![Language](https://img.shields.io/badge/language-C%2FC%2B%2B-yellow.svg)
![IDE](https://img.shields.io/badge/IDE-Arduino%20IDE-violet.svg)
![PID Control](https://img.shields.io/badge/control-PID-orange.svg)
![License](https://img.shields.io/badge/license-MIT-lightgrey.svg)

PID-based one-axis ball balancing robot using Arduino. Uses an ultrasonic sensor and servo motor to stabilize a ball at a target distance. Real-time tuning of Kp, Ki, Kd via Serial. Includes stability detection and automatic neutral reset. The system stabilizes within 30s using empirically tuned parameters.

---

## 🖼️ Project Overview

![System Diagram](https://github.com/user-attachments/assets/e33825f7-6d91-482a-a78f-bae0202825ad)

---

## 🧪 Final Project Hardware

![Hardware Picture](https://github.com/user-attachments/assets/bba11d44-11db-466e-87f8-a5fb8f7e8e28)
![Demo Picture](https://github.com/user-attachments/assets/29c81f89-d5f8-407b-894e-4baf11304527)

---

## ✨ Features

- **PID Control** for dynamic stability.
- **Real-time sensor feedback** using IR or ultrasonic sensors.
- **Servo motor-driven beam** for accurate actuation.
- **Low-cost microcontroller** implementation (Arduino/STM32).
- **Serial Monitor & LCD** integration for live debugging.

---

## 🛠️ System Architecture

![Control Flowchart](https://github.com/user-attachments/assets/263df03a-542a-4b32-a460-1ca02b46cdc8)

---

## 📽️ Demo

[Watch the demo video](https://github.com/user-attachments/assets/57880886-e9d0-444b-b646-2edba7fcaea7)

---

## 🔌 Circuit Diagram

![Circuit Diagram](https://github.com/user-attachments/assets/14326457-f14b-4431-87fc-9c9ef3d380c3)

---

## 🧾 Project Summary

This Arduino-based system controls the position of a ball on a beam using **PID control** and real-time feedback from an ultrasonic sensor. The ball is returned to a setpoint by tilting the beam via a servo motor, compensating for positional error.

---

## ⚙️ How It Works

1. **Sensing**: Ball position is measured using HC-SR04.
2. **Filtering**: Distance values are filtered (moving average).
3. **Error Calculation**: `error = distance - setpoint`
4. **PID Control**:
   - **P**: Immediate reaction to error
   - **I**: Compensates for past errors
   - **D**: Predicts upcoming trends
5. **Servo Adjustment**: Beam angle is changed proportionally to the PID output.
6. **Stability Monitoring**: If error stays < 3 cm for 2.5 seconds, servo resets to neutral.

---

## 🧠 Control Strategy

A classic PID (Proportional-Integral-Derivative) loop is implemented:

```
Error = Desired Position - Actual Position  
PID Output = Kp * Error + Ki * ∫Error + Kd * (dError/dt)
```

Final tuned values:
- `Kp = 5`
- `Ki = 0.01`
- `Kd = 9`

### Main Code Logic

```cpp
error = distance - setpoint;
integral += error;
derivative = error - previous_error;
output = -(Kp * error + Ki * integral + Kd * derivative);
servoAngle = constrain(neutralAngle + output * 1.2, 0, 180);
myServo.write(servoAngle);
```

- PID gains can be tuned via Serial commands.
- System resets to neutral if error stabilizes or ball goes out of range.

---

## 🧪 PID Tuning via Serial Monitor

```text
Kp 5         → Set proportional gain  
Ki 0.01      → Set integral gain  
Kd 9         → Set derivative gain  
neutral 34   → Set neutral servo angle  
servo -20    → Move servo from neutral
```

---

## 🔩 Hardware Components

| Component            | Description                                                             |
|----------------------|-------------------------------------------------------------------------|
| **Beam**             | 30–50 cm acrylic/wood beam, pivot-mounted                               |
| **Ball**             | Lightweight ping-pong ball                                              |
| **Servo Motor**      | SG90 (light load) or MG996R (high torque)                               |
| **Position Sensor**  | - IR (GP2Y0A21) for analog<br>- Ultrasonic (HC-SR04) for digital timing |
| **Microcontroller**  | - Arduino Uno (simple)<br>- STM32 Blue Pill (faster)                    |
| **Power Supply**     | 5V via USB or external battery pack                                     |
| **Optional Display** | 16x2 I2C LCD (PID values, position, error)                              |
| **Accessories**      | Breadboard, jumpers, brackets, bearings, screws, etc.                   |

---

## 📁 Repository Contents

- `PID-Controlled-One-Axis-Ball-Balancing-System.ino` – Core Arduino code  
- `Project_Proposal.pdf` – Initial planning  
- `Project_Presentation.pptx` – Final presentation  
- `Project_Report.pdf` – Detailed documentation  
- `Demo Video.mp4` – Final working demo  
- `Cicuit Diagram.jpg` – Wiring reference (check spelling)  
- `Project Picture.jpg` – Setup image  

---

## 🧰 Tech Stack

- Arduino Uno  
- HC-SR04 Ultrasonic Sensor  
- SG90 Servo Motor  
- Serial Monitor for real-time control  
- PID control logic (manual tuning)

---

## ✅ Results

- System stabilized within **30 seconds** of a disturbance  
- PID parameters: `Kp = 5`, `Ki = 0.01`, `Kd = 9`  
- Stability detection reduces unnecessary movement

---

## 🚀 Future Enhancements

- 2-axis (X-Y) balancing system  
- Vision-based tracking using OpenCV or IR grid  
- Self-tuning PID or ML-based control  
- Wireless monitoring via Bluetooth/Wi-Fi  
- MATLAB or Python-based simulation-first approach

---

## 👨‍💻 Authors

This project was created at **National University of Sciences & Technology (NUST)** as part of the **Linear Control Systems** course project by:

- Awais Asghar  
- Muhammad Ashar Javid  
- Ameer Hamza  
- Muhammad Hammad Sarwar  

---

## 📜 License

This project is licensed under the **MIT License**. See `LICENSE` file for details.

---

## 📚 References

- Åström & Hägglund, *Advanced PID Control*, ISA  
- Ogata, *Modern Control Engineering*, Pearson  
- Arduino PID Library  
- MathWorks MATLAB Control Toolbox  
