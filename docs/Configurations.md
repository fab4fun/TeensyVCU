# Configurations

| Option | Board Circuit Device | MCU Interface | Fcn Select by Jumper | PIN | VCU Fcn | Shifter Fcn | Engine/Exhaust Fcn | HV Battery Fcn | Aero Fcn |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Controller | Teensy4.1/ Teensy4.0 |  |  |  | 4.1 | 4.0 | 4.0 | ? | 4.1 |
| CAN | CAN Transceiver SN65HVD230 | CAN1 |  | J1_11 J1_12 | y | y | y | y | y |
| CANFD/RS232 | CAN Transceiver TCAN334GDR/ RS232 Transceiver SP3232EEN | CAN3/RX2TX2 | J2/J3 (2-3)/ J2/J3 (1-2) | J2_2 J2_3 | CANFD | RS232 Gear Indicator |  | RS232 |  |
| ISOL-CAN | CAN Transceiver ISO1050 | CAN2 |  | J2_11 J2_12 | y |  |  | y |  |
| GPS |  | RX8/TX8 |  | J9 | y |  |  |  |  |
| LS1 | MOSFET N-CH 400V 10A IRF740 | DO28 |  | J2_7 |  |  | Y | ? |  |
| LS2/ LS3 | N-CH IRF740/ PN2222A | DO29/DO18 | J4 (1-2)/ J4 (2-3) | J2_8 | LS2 | Gate Servo (5V pullup R64= 1K) | LS3 (12V pullup) | LS2? |  |
| LS4 | BJT TRANS NPN 40 V 600 mA PN2222A | DO19 |  | J2_6 |  | Engage Servo (5V pullup R65=1K) | Y (12V pullup) |  |  |
| DIG1/VR1 | OPTO LTV847S/ VR SENSE AMP LM1815N | DI9 |  | J1_7 | VR1 | ShiftUp Button | VR1 |  |  |
| DIG2/VR2 | OPTO LTV847S/ VR SENSE AMP LM1815N | DI10 |  | J1_8 | VR2 (frontwheel speeds) | ShiftDown Button | VR2 (rear wheel speeds) |  |  |
| DIG3 | OPTO LTV847S | DI11 |  | J1_9 |  | Neutral Button | Fuel Pump signal from ECM? |  |  |
| 5V_ISO/ 5V_Ref | CAN Transceiver ISO1050/ OPTO LTV847S | / A7 | J5 open/ J5 close | J2_1 | ISOL CAN supply (open J5) | Clutch 5V (close jumper J5) | Fuel Pressure 5V (close jumper J5) |  |  |
| GND_ISO/ GND_Ref | CAN Transceiver ISO1050/ | GND | J6 open/ J6 close | J2_10 | ISOL CAN ground (open J6) | Clutch Ref (close jumper J6) | Fuel Pressure Ref (close jumper J6) |  |  |
| AN1 | OPAMP 0-5V MCP604 | A2 |  | J2_4 | Brake Pressure | Clutch Motor Current (T4.0) | Fuel Pressure |  |  |
| AN2 | OPAMP 0-5V MCP604 | A3 |  | J2_5 |  | Clutch Pedal Position | Fuel Temp? |  |  |
| AN3 | OPAMP 0-5V MCP604 | A6 |  | J2_9 |  | Servo Current (T4.0) |  |  |  |
| GND |  | GNDD |  | J1_10 | Brake Pressure Ref |  |  |  |  |
| HB1/HS1/LS1 | MOSFET N-CH 60V 13A/40A AOD442G | DO2/6 |  | J1_2 |  | HB1 (clutch DC mtr) | Fuel Pump? (HS1) |  | Y |
| HB2/HS2/LS2 | MOSFET N-CH 60V 13A/40A AOD442G | DO3/14 |  | J1_3 |  | HB2 (clutch DC mtr Rev) |  |  | Y |
| HB3/HS3/LS3 | MOSFET N-CH 60V 13A/40A AOD442G | DO4/15 |  | J1_4 |  | Servo Power Return(LS only) |  |  | Y |
| HS4 | MOSFET P-CH 55V 31A IRF5305S (12V_SW Supply) | DO5 |  | J1_6 |  | Servo Power? |  | y |  |
| 12V SW | OPTO LTV847S | DI12 |  | J1_5 |  |  | Y |  |  |
| CURR1 | SENSOR CURRENT HALL 20A ACS725 | AN10 |  |  |  | Y(T4.0 jumper to AN3) | Fuel Pump Current (jumper to AN3) |  | Y |
| CURR2 | SENSOR CURRENT HALL 20A ACS725 | AN11 |  |  |  |  |  |  | Y |
| CURR3 | SENSOR CURRENT HALL 20A ACS725 | AN12 |  |  |  | Y(T4.0 jumper to AN6) |  |  | Y |
