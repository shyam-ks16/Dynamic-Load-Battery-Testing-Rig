# Dynamic-Load-Battery-Testing-Rig
Modular drone battery testing platform with programmable MOSFET-based load simulation, real-time monitoring, and structured telemetry logging. Supports realistic drone mission profiles, safe charging/discharging, and data generation for battery analytics and AI-driven evaluation.

## Overview

The Dynamic Load Battery Testing Rig is a modular battery characterization platform designed for drone and UAV battery qualification. The system performs programmable charge/discharge testing, per-cell monitoring, dynamic load simulation, and structured telemetry logging for battery performance analysis.

The platform focuses on realistic drone-representative load profiles instead of conventional constant-current testing, enabling more accurate evaluation of battery behavior under real operating conditions.

---

# Problem Statement

Existing battery testing methods for drone batteries are limited by:

* Static constant-current load testing
* Lack of realistic drone load simulation
* Inability to monitor per-cell behavior during operation
* Poor visibility into internal resistance and degradation
* High cost of commercial battery qualification systems

As a result, battery packs are often deployed without meaningful qualification or health validation.

---

# Proposed Solution

The Dynamic Load Battery Testing Rig combines:

* Programmable MOSFET-based electronic load
* Per-cell voltage monitoring
* Real-time current and temperature sensing
* Timestamped telemetry logging
* Drone mission-profile-based discharge simulation

The system supports realistic testing of LiPo/Li-ion battery packs under controlled and repeatable conditions.

---

# System Architecture

## High-Level Components

### Raspberry Pi 4

* Dashboard/UI
* Dataset/profile generation
* Data logging & analytics

### STM32 Controller

* MOSFET load control
* Real-time current regulation
* Safety monitoring

### Sensor Subsystem

* INA226 per-cell monitoring
* NTC thermistors
* Voltage and current sensing

### Programmable Load

* MOSFET-based dynamic electronic load
* PWM-controlled current sink

---

# Key Features

* Dynamic drone load profile simulation
* Per-cell voltage monitoring
* Real-time telemetry logging
* Automated safety shutdown
* Programmable current control
* Expandable modular architecture
* JSON/CSV dataset generation
* Repeatable and controlled testing environment

---

# Load Profile Simulation

The system simulates realistic drone behaviors such as:

* FPV throttle spikes
* Surveillance cruising
* Agricultural drone operation
* Ramp and pulse loads

Load profiles are generated as timestamped current-reference datasets and executed using closed-loop MOSFET control.

---

# Data Logging

The platform records:

* Pack voltage
* Per-cell voltages
* Current
* Temperature
* Event markers
* Load profile state

Data is timestamped and exported in JSON/CSV formats for further analytics and AI/ML processing.

---

# Safety Features

* Overcurrent protection
* Overtemperature shutdown
* Undervoltage cutoff
* Fault logging
* Controlled load disconnect

---

# Prototype Status

Prototype validated on:

* 3S LiPo battery pack

Validated subsystems:

* Per-cell sensing
* Current regulation
* PWM load control
* Data logging
* Safety shutdown

---

# Future Scope

* 4S–13S battery support
* 90A multi-phase discharge system
* Multi-channel parallel testing
* Flight log replay
* Automated profile generation
* Advanced analytics integration

---

# Tech Stack

## Hardware

* STM32F303
* Raspberry Pi 4
* INA226
* IRFP4568
* UCC27524
* LM393

## Software

* Python
* Dashboard Interface
* UART Communication
* JSON/CSV Telemetry

---

# Project Status

Prototype Phase — Active Development
