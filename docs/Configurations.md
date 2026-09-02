# Configurations

| Option | Board Circuit Device | MCU Interface | Fcn Select by Jumper | PIN | VCU Fcn | Shifter Fcn | Engine/Exhaust Fcn | HV Battery Fcn | Aero Fcn | eCharger BoostController Fcn |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Controller | Teensy4.1/ Teensy4.0 |  |  |  | 4.1 | 4.0 | 4.0 | ? | 4.1 | 4.1 |
| CAN | CAN Transceiver SN65HVD230 | CAN1 |  | J1_11 J1_12 | y | y | y | y | y | y (shared vehicle CAN) |
| CANFD/RS232 | CAN Transceiver TCAN334GDR/ RS232 Transceiver SP3232EEN | CAN3/RX2TX2 | J2/J3 (2-3)/ J2/J3 (1-2) | J2_2 J2_3 | CANFD | RS232 Gear Indicator |  | RS232 |  |  |
| ISOL-CAN | CAN Transceiver ISO1050 | CAN2 |  | J2_11 J2_12 | y |  |  | y |  | y (to Sevcon inverter) |
| GPS |  | RX8/TX8 |  | J9 | y |  |  |  |  |  |
| LS1 | MOSFET N-CH 400V 10A IRF740 | DO28 |  | J2_7 |  |  | Y | ? |  | N75 Diverter/Boost Valve |
| LS2/ LS3 | N-CH IRF740/ PN2222A | DO29/DO18 | J4 (1-2)/ J4 (2-3) | J2_8 | LS2 | Gate Servo (5V pullup R64= 1K) | LS3 (12V pullup) | LS2? |  | COOLPUMP_LS (as LS2) |
| LS4 | BJT TRANS NPN 40 V 600 mA PN2222A | DO19 |  | J2_6 |  | Engage Servo (5V pullup R65=1K) | Y (12V pullup) |  |  | Sevcon Key Relay LS Driver |
| DIG1/VR1 | OPTO LTV847S/ VR SENSE AMP LM1815N | DI9 |  | J1_7 | VR1 | ShiftUp Button | VR1 |  |  |  |
| DIG2/VR2 | OPTO LTV847S/ VR SENSE AMP LM1815N | DI10 |  | J1_8 | VR2 (frontwheel speeds) | ShiftDown Button | VR2 (rear wheel speeds) |  |  |  |
| DIG3 | OPTO LTV847S | DI11 |  | J1_9 |  | Neutral Button | Fuel Pump signal from ECM? |  |  |  |
| 5V_ISO/ 5V_Ref | CAN Transceiver ISO1050/ OPTO LTV847S | / A7 | J5 open/ J5 close | J2_1 | ISOL CAN supply (open J5) | Clutch 5V (close jumper J5) | Fuel Pressure 5V (close jumper J5) |  |  | ISOL CAN supply, from Sevcon (open J5) |
| GND_ISO/ GND_Ref | CAN Transceiver ISO1050/ | GND | J6 open/ J6 close | J2_10 | ISOL CAN ground (open J6) | Clutch Ref (close jumper J6) | Fuel Pressure Ref (close jumper J6) |  |  | ISOL CAN ground, from Sevcon (open J6) |
| AN1 | OPAMP 0-5V MCP604 | A2 |  | J2_4 | Brake Pressure | Clutch Motor Current (T4.0) | Fuel Pressure |  |  | Throttle/Bypass TPS1 |
| AN2 | OPAMP 0-5V MCP604 | A3 |  | J2_5 |  | Clutch Pedal Position | Fuel Temp? |  |  | Throttle/Bypass TPS2 |
| AN3 | OPAMP 0-5V MCP604 | A6 |  | J2_9 |  | Servo Current (T4.0) |  |  |  |  |
| GND |  | GNDD |  | J1_10 | Brake Pressure Ref |  |  |  |  |  |
| HB1/HS1/LS1 | IR2104 half-bridge driver + MOSFETs | DO2 (IN) / DO6 (~SD) |  | J1_2 |  | HB1 (clutch DC mtr) | Fuel Pump? (HS1) |  | Y | Throttle/Bypass Motor+ Drive |
| HB2/HS2/LS2 | IR2104 half-bridge driver + MOSFETs | DO3 (IN) / DO14 (~SD) |  | J1_3 |  | HB2 (clutch DC mtr Rev) |  |  | Y | Throttle/Bypass Motor- Drive |
| HB3/HS3/LS3 | IR2104 half-bridge driver + MOSFETs | DO4 (IN) / DO15 (~SD) |  | J1_4 |  | Servo Power Return(LS only) |  |  | Y |  |
| HS4 | MOSFET P-CH 55V 31A IRF5305S (12V_SW Supply) | DO5 |  | J1_6 |  | Servo Power? |  | y |  |  |
| 12V SW | OPTO LTV847S | DI12 |  | J1_5 |  |  | Y |  |  | Y |
| CURR1 | SENSOR CURRENT HALL 20A ACS725 | AN10 |  |  |  | Y(T4.0 jumper to AN3) | Fuel Pump Current (jumper to AN3) |  | Y | HB1 Current Sensing (Throttle/Bypass) |
| CURR2 | SENSOR CURRENT HALL 20A ACS725 | AN11 |  |  |  |  |  |  | Y | Shorted on board |
| CURR3 | SENSOR CURRENT HALL 20A ACS725 | AN12 |  |  |  | Y(T4.0 jumper to AN6) |  |  | Y |  |
| X3_1 | OPAMP 0-5V MCP604 | A14 |  | X3_1 |  |  | MAP_Signal (existing) |  |  | Boost Pressure Signal (MAP_Signal) |
| X3_2 |  | GNDD |  | X3_2 |  |  | GND (existing) |  |  | GND |
| X3_3 | OPAMP 0-5V MCP604 | A15 |  | X3_3 |  |  | IAT_Signal (existing) |  |  | Boost Air Temp Signal (IAT_Signal) |
| X3_4 |  | +5V |  | X3_4 |  |  | 5V_Ref (existing) |  |  | 5V Sensor Supply |

**eCharger BoostController basis (added 2026-08-31, corrected same day):** The Teensy VCU instance used here is the `TeensyEngine` sheet inside `Engine.kicad_sch` (physically the same `VehCtrl.kicad_sch` board design also instantiated at the top level as `TeensyVCU`). Confirmed via user domain knowledge: LS1/LS2-LS3/LS4/HB1/HB2 are not generic "Engine/Exhaust" functions but the eCharger's own N75 diverter/boost valve, cooling pump (`COOLPUMP_LS`), Sevcon Gen4 key relay driver, and throttle/bypass valve motor drive (`U58` "Bosch_Throttle_Body" is reused here as the boost bypass valve actuator, not the main engine throttle). No VR1/VR2 (DIG1/DIG2) connection is required. Boost pressure/temp feedback is via the existing `X3` connector (already wired to `U55` "Bosch_0261230283_T-MAP" in `Engine.kicad_sch`, not a new/free pin) — `X3_1`/`X3_3` MCU pin mapping (`A14`/`A15`) confirmed directly from the Teensy 4.1 symbol's pinout in `VehCtrl.kicad_sch` (physical pins 30/31), `X3_2` ground from the Teensy's `GNDD` net, `X3_4` from the board's regulated `+5V` rail (Teensy `VIN`, physical pin 48) — not routed through an MCU GPIO.

**H-bridge topology confirmed (2026-08-31):** HB1 and HB2 are each an independent IR2104 half-bridge driver channel (not raw MOSFET gate drive) — `IN` (DO2 for HB1, DO3 for HB2) sets that leg's output/PWM duty, `~SD` (DO6 for HB1, DO14 for HB2, active-low) is a hardware shutdown for that channel. A full H-bridge is formed by the two independent half-bridges driving the motor's two terminals (Motor+/Motor-). `CURR1`'s real Teensy pin is confirmed as `A10` (matches the `AN10` label above, no discrepancy).

**Known discrepancy (unrelated to eCharger):** this table's "Engine/Exhaust Fcn" entries for DIG1/VR1 and DIG2/VR2 (`VR1`, `VR2 (rear wheel speeds)`) do not match the current `TeensyEngine` schematic, which shows both pins floating/unconnected. Verify in KiCad directly before relying on that pre-existing note.
