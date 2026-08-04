# TeensyVCU ideas

24V tolerant Regulated input for isolated CAN

Flip GPS board for better reception?

Ethernet? Wifi? Bluetooth?

Current sense option on all HS and LS outputs

Hold-capable driver for HS on half bridges

https://www.farnell.com/datasheets/2254418.pdf

External 5V supply board?  Higher power 5V?

Use existing 5V linear to power teensy only?

-needs enable circuit

-switching PS noise acceptable?

Pull down for 12V_SW when disconnected?

Needs diode on VCC for half-bridge drivers to not backfeed circuit through DI

OR  read 12V_SW through analog input and trigger <6V

CAN3 - use 3.3V from Teensy..   And standby mode

-allows for wake on CAN

Fix ISO1044 pads

SPI for 5V tolerant A/D?

8pins -> 3pins + 2xCS

Replaces op-amps

TI TLA2518 Small, 8-Channel, 12-Bit ADC with SPI Interface and GPIOs

TI ADS7038-Q1 Small, 8-Channel, 12-Bit ADC with SPI Interface, GPIOs, and CRC

5V ISO jumper J5 in way of teensy usb port

5V pull up for LS1 and LS2 to drive additional servo

Or:  reconfigure optocoupler
