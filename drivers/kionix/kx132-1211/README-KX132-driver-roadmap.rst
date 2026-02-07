KX132 Zephyr Driver Roadmap
2023-01-16

OVERVIEW
********

This driver written in C designed to work with Zephyr RTOS, as of its releases 2.6.0 and 3.2.0.  Significant parts of this driver follow the design of STMicro's IIS2DH driver, specifically around the support of I2C and SPI bus connections and the use of Zephyr device tree macro API to allow their selection in an application project, nearly fully in device tree overlay files.

FILES AND FEATURE FACTORING
***************************

kx132-1211.h          . . . primary driver header file, defines:

                            *  Zephyr RTOS sensor data and config structures
                            *  extra Zephyr sensor channels to read KX132 specific parameters
                            *  sensor channel and sensor attribute enumerations to support Zephyr API conventions
                            *  KX132 available I2C device addresses, per TN027-Power-On-Procedure.pdf

kx132-1211.c          . . . implements Zephyr sensor APIS

                            *  _attr_set
                            *  _attr_get
                            *  _sensor_fetch
                            *  _sensor_get

                            and instantiates sensor structure instances for each KX132
                            found in project device tree.

kx132-low-level-bus-interface.h
                      . . . function pointer type defines for low level register read and write functions

kx132-i2c.[ch]        . . . low level I2C register read and write routines

kx132-spi.[ch]        . . . low level SPI register read and write routines

kx132-registers.h     . . . defines:

                            *  symbol names for sensor registers, some bit-wise flag names
                            *  prototypes for low level register read and write wrappers
                            *  prototypes for defined sensor config sequences
                            *  prototypes for sensor parameter fetch (read) routines

kx132-registers.c     . . . implements:

                            *  implements defined sensor configuration sequences
                            *  implements sensor parameter fetch (read) routines

kx132-triggers.[ch]   . . . NOT IMPLEMENTED NOT TESTED TO WORK, AS OF 2023 Q1 (note 1) - TMH

out-of-tree-drivers.h . . . enumerated driver routine return values

REFERENCES
**********

*  TN027-Power-On-Procedure.pdf                        . . . recommended power up test for all uses, I2C addressing details
*  KX132-1211-Technical-Reference-Manual-Rev-5.0.pdf   . . . register and sensor function details
*  AN092-Getting-Started.pdf                           . . . application notes on KX132 practical use configurations

*  https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/sensor/lis2dh
*  https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/sensor/lis2dh/src/main.c#L81
*  https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/basic/button

NOTES
*****

(1)  Developer attempts to configure a GPIO based interrupt for KX132 driver, within Zephyr's sensor driver framework and practices have failed as of 2022 Q4.  In other words, attempts to configure a hardware interrupt in this KX132 driver, following example of Zephyr 3.2.0 sample app for STMicro's LIS2DH sensor have so far not succeeded.  An alternative however is tested and working, to set up in one's Zephyr app a GPIO based interrupt in the application.  An example of code for this sensor interrupt solutino is exemplified in Zephyr 3.2.0 sample application named 'button'.

CONTRIBUTORS
************
This driver developed initially by Ted Havelka.
