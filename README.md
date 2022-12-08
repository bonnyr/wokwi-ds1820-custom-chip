# wokwi-ds1820-custom-chip
<img src="ds18b20.jpg" width="100" height="100"/>

[chip data sheet](https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf)

The DS18B20 Custom Chip simulates the Dallas Semi One Wire Temperature Sensor of the same name. 
It provides 16 channels of PWM control that allow driving of LEDs or small servos. 

The chip has the following pin groups

| Name         | Description                                            |
| ------------ | ------------------------------------------------------ |
| `Vdd`   | Power pin          |
| `GND`     | ground pin                        |
| `DQ`      | Data Pin, used as I/O pin           |


### Addressing
The device uses OneWire compatible addresses, configurable as described below
### OneWire Comms 
The device uses OneWire for comms with the MCU. This custom chip implements a OneWire Slave (only)

A good Arduino library to use on the Arduino side is the OneWire Server Library (TBD)


## Implementation details
This chip supports all the functions of the hardware part. Commands are listed below

### Reset 
Write only. Resets the chip to power on. 

### Search
The device responds to the search command.

[TBD]
### Read

### Match

### Skip

### Alarm Search

### Convert

### Write Scratchpad

### Read Scratchpad

### Copy

### Recall 

### Read Power Supply
The device behavior depends on the value of `deviceMode` attribute [see below](#devicemode)
When configured in Parasitic mode, the chip will pull down the wire when requested to indicate as much. When configured in Vdd mode, the chip will leave the wire floating.


## Attributes
The chip defines a number of attributes that alter the behavior of the  operation when used in Wokwi. 

| Name         | Description                                            | Default value             |
| ------------ | ------------------------------------------------------ | ------------------------- |
| <span id="deviceMode">deviceMode</span>   |  controls whether the device behaves as if it was connected to Vdd or uses parasitic voltage | "0"                 |
| <span id="deviceID">deviceID</span>   |  Specifies the 64bit device serial number. This is a string and the value should be limited to precisely 16hex digits<br>Note the serial number must be a valid id, including CRC | "9F9D876799C4F707"                 |

## Simulator examples

- [DS18B20 Custom Chip](https://wokwi.com/projects/350278641316266578)

##  Limitations and ommissions
### Convert Temperature in Parasitic mode
In this mode, the master is pulling the bus high for a period of at least 10us during which no activity 
takes place on the bus. The current implementation completes the conversion immediately and is going back 
to the init sequence. This is likely not an issue, but it deviates from the spec somewhat

