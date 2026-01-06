# Adaptive Cruise Control (ACC)

## Overview
Designed and validated a longitudinal Adaptive Cruise Control (ACC) system focused on safety, ride comfort, and robustness.

## Problem Statement
Develop a controller capable of maintaining a safe following distance while tracking a driver-set speed under varying traffic conditions.

## System Architecture
![ACC Architecture](images/acc_architecture.png)

## Control Strategy
- Longitudinal vehicle modeling
- PID-based control with feedforward
- Safety constraints for minimum following distance

## Testing & Results
- Model-in-the-Loop (MIL)
- Software-in-the-Loop (SIL)
- Hardware-in-the-Loop (HIL)

![Results](images/acc_results.png)

## Tools & Technologies
MATLAB, Simulink, C++, CarMaker, HIL
