# Exodus2025_Science
🏗️ System Architecture Overview

This repository contains the complete software stack for the TAUVER Rover's dual-science payload. The system operates on a highly distributed ROS 2 (Jazzy) framework, bridging low-level microcontroller hardware (Layer 1) to an asynchronous PyQt5 graphical command station (Layer 7) across a wireless CycloneDDS middleware layer.

The architecture is divided into two physically and logically isolated subsystems:

The Main Science Payload (Dry Lab): Geological, atmospheric, and physical metrology.

The Astrobiology Payload (Wet Lab): Fluidic pH analysis and autonomous chemical injection.

📂 Repository File Index

🏜️ 1. Main Science Payload (Dry Lab)

rover_v7_manual.ino

Role: C++ Silicon Firmware (Arduino Mega).

Function: Directly manages hardware interrupts, I2C/SPI bus polling (SCD4x CO2/Temp, MPU6050 IMU, HX711 Load Cells), and executes physical servo actuation for the rock/deep sample containment lids.

jetson_manual_bridge.py

Role: ROS 2 Hardware Translator Node.

Function: Binds to the serial UART stream, decodes raw JSON sensor payloads, and publishes strongly-typed data to the /science/ ROS 2 namespace at 20Hz.

sensor_farm_v6_dual.py

Role: Primary GUI Command Station.

Function: Thread-safe PyQt5 dashboard. Renders live telemetry and injects hardware commands (FSM overrides, lid actuation, scale taring) into the DDS network.

🦠 2. Astrobiology Subsystem (Wet Lab)

astro_bio_arduino.ino

Role: C++ Silicon Firmware (Arduino).

Function: Executes low-level ADC sampling for pH electrode voltages and triggers H-Bridge PWM signals for precise bidirectional DC peristaltic pump control.

astro_jetson_bridge.py

Role: ROS 2 Wet-Lab Translator Node.

Function: Publishes live pH metrology and pump state integers to the /astro/ namespace. Subscribes to string commands for hardware triggering.

astrobio_flight_deck.py

Role: Advanced Metrology & Statistical GUI.

Function: PyQt5 dashboard featuring temporal drift plotting and a built-in 2-Sigma Anomaly Rejection Engine. Utilizes Chauvenet's Criterion to dynamically filter out fluidic noise/bubbles and calculates publish-grade Standard Error ($SE$) for chemical readings.

📡 3. Network Infrastructure & Overrides

cyclonedds.xml

Role: Layer 2 Enterprise Switch Bypass.

Function: Custom RTPS middleware configuration. Forces ROS 2 to abandon UDP Multicast discovery (which is heavily blocked by enterprise IGMP Snooping) in favor of point-to-point Unicast, ensuring system survivability on highly restrictive networks.

🔌 ROS 2 API Contract (For Systems Integration)

The payload operates completely independent of the Python GUIs. Systems integration can tap directly into the telemetry by subscribing to the following topics:

Main Science Topics (std_msgs):

/science/sensors/co2 (Int32)

/science/sensors/temp (Float32)

/science/sensors/rock_mass (Float32)

/science/sensors/pitch (Float32)

/science/manual_cmds (String) -> Publishes actuation targets (e.g., "ROCK_OPEN")

Astrobiology Topics (std_msgs):

/astro/sensors/ph (Float32)

/astro/sensors/pump (Int32)

/astro/manual_cmds (String) -> Publishes pump injection targets (e.g., "PUMP_3000")

🚀 Quick Start (Deployment)

For the comprehensive, step-by-step physical deployment and network mapping protocol, please refer to the tauver_dummy_proof_manual.md included in this repository.

Property of the TAUVER Science Team.
