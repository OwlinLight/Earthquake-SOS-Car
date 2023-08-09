# SOS Car System with Rescue Team Recognition

The SOS Car System is a project aimed at developing an autonomous vehicle capable of navigating its way out of dangerous situations, sending distress signals, identifying rescue teams, and guiding them back to the location. This README provides an overview of the project, its features, and deployment details.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Technology Stack](#technology-stack)
- [Installation](#installation)
- [Usage](#usage)
- [Rescue Team Detection](#rescue-team-detection)
- [Motor Control and Routing](#motor-control-and-routing)
- [Deployment](#deployment)

## Introduction

The SOS Car System is designed to provide a self-contained solution for emergency situations where a vehicle needs to escape hazardous environments and seek help. The system is built on the Raspberry Pi 3 platform running Linux (Ubuntu) and utilizes various technologies to achieve its objectives.

:tv: [Watch the SOS Car System in action](https://youtu.be/3ZVTaL1G0AA)

## Features

- Autonomous navigation in dangerous environments.
- Distress signal activation and transmission.
- Recognition of rescue team personnel.
- Guiding rescue teams back to the vehicle's location.

## Technology Stack

- Raspberry Pi 3
- Linux (Ubuntu 20.04)
- Python
- OpenCV for rescue team detection
- Motor control for vehicle movement
- Routing algorithms for navigation

## Installation

1. Clone this repository to your Raspberry Pi 3.
2. `cd 1.Code`, Install OpenCV for Python.
3. run command `python run.py`.
4. Connect the necessary hardware components, including motors and sensors, to the Raspberry Pi.

## Usage

1. Run the main application script to start the SOS Car System.
2. The system will assess its environment and autonomously navigate to a safe location.
3. In case of danger, the distress signal can be activated manually or automatically.
4. The rescue team recognition module will identify approaching rescue personnel.

## Rescue Team Detection

Rescue team detection is achieved using OpenCV's cascade classifier. The system is trained to recognize specific visual cues associated with rescue team members. Upon detection, the system can alter its behavior to guide the rescue team to the vehicle's location.

## Motor Control and Routing

The motor control system is responsible for the movement of the vehicle. It interacts with the routing algorithms to ensure safe and efficient navigation. The routing algorithm takes into account the vehicle's current position, destination, and environmental data to determine the optimal path.

## Deployment

Deploying the SOS Car System involves setting up the Raspberry Pi 3 with the required software components. Ensure that all hardware components are properly connected and calibrated before deploying the system in hazardous environments.

---

For more information, issues, and inquiries, please contact us at 201806040620@zjut.edu.cn