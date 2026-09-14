# Autonomous Target-Tracking Mobile Robot (Visual Servoing)

Real-time vision-based control loop implemented on Raspberry Pi for target tracking using monocular computer vision and differential PWM motor control.

## Overview
- **Perception:** Real-time acquisition via `Picamera2` (320x240 @ 10 Hz), preprocessing (Gaussian/median blur, color space conversion), and circle detection using Hough Transform (`cv2.HoughCircles`).
- **Estimation & Filtering:** Outlier rejection and temporal moving average window ($N=10$) on target coordinates $(x, y, r)$ to ensure tracking stability under vibration.
- **Control Architecture:** Closed-loop proportional controller computing heading error ($\text{err}_x$) and distance error ($\text{err}_r$) coupled with differential PWM signals on dual DC motors.
- **Safety & Alerts:** Hardware threshold alerting via GPIO buzzer and clean fail-safe termination.

## Hardware Stack
- Raspberry Pi (Raspberry Pi OS)
- Raspberry Pi Camera Module (Picamera2)
- 2WD / 4WD Differential Chassis with DC Motors + H-Bridge Driver (PWM)
- Active Buzzer (GPIO alert)

## Quick Start
Clone the repository:
```bash
git clone [https://github.com/](https://github.com/)<TON_PSEUDO>/mobile-robot-visual-servoing.git
cd mobile-robot-visual-servoing
python3 main.py
