# 2026TriShooter - FRC Team 6002

## About

This repository contains the official robot code for FRC Team 6002, "ZooBOTix", for the 2026 FIRST Robotics Competition season

## Dependencies

### Core Libraries

- WPILib: Primary robot framework
- REVLib: Spark Max & Spark Flex motor controller API
- PathPlannerLib: Autonomous path generation and following
- StudicaLib: NavX2 Gyro API

### Logging & Simulation

- AdvantageKit: High‑resolution logging, replay, telemetry visualization
- AdvantageScope: Real‑time log viewer and analysis tool
- URCL (Unofficial REV Compatible Logger): Automatic Logging of all CAN traffic from REV motor controllers to NetworkTables
- MapleSim: Physics‑based drivetrain modeling and simulation

### Vision

- PhotonVision: April Tag Detection & Localization 

## CAN Devices

| Subsystem |       Device      | CAN ID | Motor      |
|-----------|-------------------|--------|------------|
| Drive     | Front Left Drive  | 62     | NEO VORTEX |
| Drive     | Front Right Drive | 15     | NEO VORTEX |
| Drive     | Back Left  Drive  | 8      | NEO VORTEX |
| Drive     | Back Right Drive  | 11     | NEO VORTEX |
| Drive     | Front Left Turn   | 6      | NEO 550    |
| Drive     | Front Right Turn  | 14     | NEO 550    |
| Drive     | Back Left  Turn   | 9      | NEO 550    |
| Drive     | Back Right Turn   | 10     | NEO 550    |
| Intake    | Intake Extender   | 7      | NEO V1     |
| Intake    | Roller Lead       | 12     | NEO VORTEX |
| Intake    | Roller Follower   | 1      | NEO VORTEX |
| Shooter   | Left Shooter      | 3      | NEO V1     |
| Shooter   | Middle Shooter    | 4      | NEO V1     |
| Shooter   | Right Shooter     | 17     | NEO V1     |
| Hood      | Hood              | 2      | NEO 550    |
| Conveyor  | Conveyor          | 13     | NEO V1     |
| Kicker    | Kicker            | 16     | NEO V1     |

**Note:** All motor controllers use a CAN ID matching their PDH slot. The only exception is PDH slot 0, which uses CAN ID 62 because motor controllers cannot be assigned CAN ID 0.

## Build, Deploy, & Simulate

### Building the code

You can build the robot code in two ways

- Using WPILib VsCode:
 Press Ctrl + Shift + P, then search for 
 `WPILib: Build Robot Code`
- Using Gradle
`./gradlew build`

### Deploying the code

Similarly there are two ways to deploy the code to a robot

- Using WPILib VsCode: Press Ctrl + Shift + P, then search for
 `WPILib: Deploy Robot Code`

- Using Gradle `./gradlew deploy`

### Simulating the code

You can also simulate the robot code with two methods

- Using WPILib VsCode:
Press Ctrl + Shift + P, search for 
`WPILib: Simulate Robot Code`, and then choose:
  - Sim GUI if you don’t have the NI Game Tools installed
  - Real Driver Station if you want to use the official NI Driver Station

- Using Gradle: `./gradlew simulateJava`