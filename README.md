# Elliot: Autonomous Self-Driving Car

An affordable autonomous vehicle platform built using Raspberry Pi and LEGO EV3. Integrates YOLO-based object detection and U-Net segmentation for real-time navigation and emergency response.

## 🚗 Key Features
- YOLOv11-based traffic sign detection
- YOLOv11-based fire and smoke detection
- U-Net road segmentation for lane navigation
- Sensor fusion (camera, ultrasonic, gyroscope)
- Manual override system with emergency drone dispatch

## 🧠 Model Performance
| Model               | mAP@50 | mAP@50-95 | IoU    |
|--------------------|--------|-----------|--------|
| Traffic Sign YOLO  | 82.7%  | 66.6%     | —      |
| Fire/Smoke YOLO    | 67.5%  | 38.7%     | —      |
| U-Net Segmentation | —      | —         | 0.97 (train), 0.95 (val) |

## 🛠 Hardware
- Raspberry Pi 4 Model B
- LEGO Mindstorms EV3 motors and chassis
- Pi Camera module
- HC-SR04 Ultrasonic sensors
- MPU6050 Gyroscope
- Drone (optional, emergency only)


## ⚙️ Installation
```bash
git clone https://github.com/BasharLoudah/elliot-autonomous-car.git
cd elliot-autonomous-car
pip install -r requirements.txt
