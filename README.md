# 🤖 3DOF Robotic Arm – Inverse Kinematics (Arduino)

This project implements a **3-DOF robotic arm** using Arduino and servo motors, where the arm moves to a desired **(X, Y, Z)** position using an analytical inverse kinematics approach.

The user inputs coordinates through the Serial Monitor, and the system computes joint angles and drives the servos accordingly.

---

## 🚀 Features

- Control using **Cartesian coordinates (X, Y, Z)**
- Real-time **inverse kinematics computation**
- Serial-based input interface
- Handles **unreachable positions**
- Modular IK solver (`solveAngles()`)

---

## 🧰 Hardware Used

- Arduino Uno  
- 3 Servo Motors  
  - Yaw (Base)  
  - Shoulder  
  - Elbow  

---

## 🔌 Pin Configuration

| Joint      | Pin |
|------------|-----|
| Yaw        | 9   |
| Shoulder   | 2   |
| Elbow      | 10  |

---

## 📐 Robot Parameters

```cpp
float r1 = 74.63;     // Base height offset
float r2 = 111.945;   // Shoulder link length
float r3 = 130.05;    // Elbow link length
```
## 🧠 Inverse Kinematics

### Base Angle
θ₁ = atan2(y, x)

### Coordinate Conversion
r = sqrt(x² + y²)  
z_eff = z - r1  

### Joint Angles
K = (r² + z² + l₃² - l₂²) / (2l₃)  
R = sqrt(r² + z²)  

θ₃ = α + acos(K / R)  
θ₂ = atan2(z - l₃·sinθ₃, r - l₃·cosθ₃)  

> This implementation uses **Elbow-Down configuration**

## ▶️ How to Use

1. Upload the code to Arduino  
2. Open Serial Monitor (9600 baud)  
3. Enter:  
100 50 120  
4. Press Enter

## ⚠️ Current Issues

- No restriction on **workspace limits**, so the arm may try to reach unreachable positions  
- Servo angles are only constrained numerically (0°–180°), not based on **actual mechanical limits**  
- Lack of **calibration offsets**, leading to positioning inaccuracies  
- Sudden movements due to **no smooth motion control**

---

## 🔮 Future Improvements

- Implement **workspace boundary checks** to ensure safe and reachable positions  
- Add **mechanical limit constraints** for each joint to prevent damage  
- Include **proper calibration for each servo** to improve accuracy  
- Add a **gripper (4th DOF)** for basic pick-and-place functionality  
