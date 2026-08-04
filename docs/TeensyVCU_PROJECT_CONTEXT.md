# TeensyVCU Project Context

## Overview
The **TeensyVCU** project is a modular vehicle control unit built around the Teensy 4.x microcontroller family. It interfaces with automotive subsystems such as CAN, RS‑232, Ethernet, GPS, and various power‑sensing circuits. The design emphasizes flexibility through jumper‑selectable configurations, isolated CAN lines, and support for both high‑voltage (HV) and low‑voltage (LV) signals.

## Core Hardware Components
- **MCU**: Teensy 4.1 or 4.0 (configurable via the *Controller* option). The MCU runs at 600 MHz and supplies up to 3.3 V logic for peripherals.
- **CAN Transceivers**:
  - CAN1 – SN65HVD230 (standard CAN, 1 Mbps)
  - CAN2 – ISO1050B (isolated CAN, optional)
  - CAN3 – TCAN334GDR (CAN‑FD)
- **Power Supply**:
  - TLV75733PDBVR buck regulator rated 1 A for the 3.3 V rail.
  - External 5 V supply options are considered; a linear regulator on the Teensy board provides ~200 mA.
- **Analog Front‑End**: Eight‑channel 12‑bit ADCs (TI TLA2518 or ADS7038‑Q1) replace legacy op‑amps for sensor read‑outs.
- **Driver FETs**: N‑CH MOSFETs (IRF740, AOD442G) and a P‑CH IRF5305S for high‑voltage switching.

## Configuration Matrix
The `Configurations.md` table maps each subsystem to its physical pinout, MCU interface, and functional role. Key takeaways:
- **Jumper selections** (e.g., J5 open/close) enable isolated CAN supply or 5 V reference for clutch‑related signals.
- **Signal roles** are split between VCU, shifter, engine/exhaust, HV battery, and aero subsystems.
- **Current sense** is routed through dedicated Hall sensors (ACS725) to the ADC inputs AN10–AN12.


## Design Intent, Future Work and Quick Notes
 - **Isolated CAN**: The ISO1050 transceiver and dedicated power/ground pins (J5/J6) provide galvanic isolation for safety‑critical data.
- **Wake‑on‑CAN**: Standby mode (20 µA) allows the MCU to be powered down until a CAN message wakes it.
- **High‑voltage handling**: 12 V switching is isolated via an optocoupler and monitored through DI12.
- **Scalable ADCs**: Replacing op‑amps with SPI‑based 12‑bit ADCs reduces board space and improves accuracy.
- **Future extensions**: Ethernet, Wi‑Fi, or Bluetooth modules are considered for enhanced connectivity.

---
For detailed pin assignments, refer to `Configurations.md`. For power calculations, see `Power_Requirements.md`.