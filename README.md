# Deprecated
1. Better to use more preciese sensor: MH-Z19B
2. GeorgK had some mistakes in formulas. Looks like https://github.com/Gazz-yniere/MQ135 fork is more correct one. 

----

# Connection
Raspberry doesn't have ADC. So we need ADS1115 or MCP3428 (Analog-to-Digital Converter) between sensor's analogue output and rpi.

# MQ135 (Gas Sensor)

Datasheets:
https://www.olimex.com/Products/Components/Sensors/SNS-MQ135/resources/SNS-MQ135.pdf
http://www.ti.com/lit/ds/symlink/ads1115.pdf

Application:
They are used in air quality control equipments for buildings/offices, are suitable for detecting of NH3, NOx, alcohol, Benzene, smoke, CO2, etc. Measurement in PPM.

Original creator of this library: https://github.com/GeorgK/MQ135
