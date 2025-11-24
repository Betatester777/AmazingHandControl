# Potentiometer SCSCL Servo – Memory Table Description

## Table of Contents

- [Potentiometer SCSCL Servo – Memory Table Description](#potentiometer-scscl-servo--memory-table-description)
- [1. Servo Communication Protocol](#1-servo-communication-protocol)
- [2. Servo Memory Table Definition](#2-servo-memory-table-definition)
  - [2.1 Version Information](#21-version-information)
  - [2.2 EPROM Configuration](#22-eprom-configuration)
  - [2.3 SRAM Control](#23-sram-control)
  - [2.4 SRAM Feedback](#24-sram-feedback)
  - [2.5 Factory Parameters](#25-factory-parameters)
- [3. Special Byte Description](#3-special-byte-description)
  - [3.1 Servo Phase](#31-servo-phase)
  - [3.2 Servo Status](#32-servo-status)
  - [3.3 Unload Conditions](#33-unload-conditions)
  - [3.4 LED Alarm Conditions](#34-led-alarm-conditions)

---

## 1. Servo Communication Protocol

The servo uses the **FT-SCS custom protocol**.  

- Default baud rate: **1 Mbps or 500 kbps**
- Physical layer: **TTL single bus**
- Data bits: **8**
- Parity: **none**
- Stop bits: **1**
- Configurable baud range: **38 400 ~ 1 Mbps (500 k)**
- Default communication address (ID): **1**

Protocol reference:  
[FT-SCS Custom Protocol](http://doc.feetech.cn/#/tiaozhunlujingft?srcType=FT-SCS-Protocol-41ad23fe8a244712ba160b93)

---

## 2. Servo Memory Table Definition

> If a function address uses a 2-byte value, the **high byte** is stored at the **lower address**, and the **low byte** at the **higher address** (big-endian within the table).

---

### 2.1 Version Information

| Address DEC | Address HEX | Function Name           | Bytes | Default | Access | Range | Unit | Description                            |
|-------------|-------------|-------------------------|-------|---------|--------|-------|------|----------------------------------------|
| 0           | 0x00        | Firmware major version  | 1     | –       | R      |       |      |                                        |
| 1           | 0x01        | Firmware minor version  | 1     | –       | R      |       |      |                                        |
| 2           | 0x02        | END                     | 1     | 1       | R      |       |      | `1` indicates big-endian storage       |
| 3           | 0x03        | Servo major version     | 1     | –       | R      |       |      |                                        |
| 4           | 0x04        | Servo minor version     | 1     | –       | R      |       |      |                                        |

---

### 2.2 EPROM Configuration

| Address DEC | Address HEX | Function Name                 | Bytes | Default | Access | Range       | Unit  | Description                                                                                                                                         |
|-------------|-------------|-------------------------------|-------|---------|--------|-------------|-------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 5           | 0x05        | Servo ID                      | 1     | 1       | R/W   | 0 ~ 253     | ID    | Unique main ID on the bus                                                                                                                           |
| 6           | 0x06        | Baud rate                     | 1     | 0       | R/W   | 0 ~ 7       | –     | 0–7 represent baud: 1000000(0), 500000(1), 250000(2), 128000(3), 115200(4), 76800(5), 57600(6), 38400(7)                                           |
| 7           | 0x07        | Undefined                     | 1     | –       | R/W   | –           | –     | –                                                                                                                                                   |
| 8           | 0x08        | Status return level           | 1     | 1       | R/W   | 0 ~ 1       | –     | 0: only READ and PING return status; 1: all commands return status packets                                                                          |
| 9           | 0x09        | Minimum angle limit           | 2     | 20      | R/W   | 0 ~ 1023    | steps | Minimum operating angle; must be less than max angle. If **min angle = max angle = 0** → motor (continuous rotation) mode                          |
| 11          | 0x0B        | Maximum angle limit           | 2     | 1003    | R/W   | 1 ~ 1023    | steps | Maximum operating angle; must be greater than min angle. If **min angle = max angle = 0** → motor mode                                             |
| 13          | 0x0D        | Max temperature limit         | 1     | 70      | R/W   | 0 ~ 100     | °C    |                                                                                                                                                     |
| 14          | 0x0E        | Maximum input voltage         | 1     | –       | R/W   | 0 ~ 254     | 0.1 V | If **max input voltage = min input voltage = 0**, voltage feedback is disabled                                                                      |
| 15          | 0x0F        | Minimum input voltage         | 1     | 40      | R/W   | 0 ~ 254     | 0.1 V | If **max input voltage = min input voltage = 0**, voltage feedback is disabled                                                                      |
| 16          | 0x10        | Maximum torque                | 2     | 1000    | R/W   | 0 ~ 1000    | 0.1%  | On power-up this value is copied to address 48 (torque limit)                                                                                       |
| 18          | 0x12        | Phase                         | 1     | –       | R/W   | 0 ~ 254     | –     | Special function byte; do not modify without specific need                                                                                          |
| 19          | 0x13        | Unload conditions             | 1     | –       | R/W   | 0 ~ 254     | –     | Each bit enables/disables a corresponding protection (see [3.3](#33-unload-conditions))                                                             |
| 20          | 0x14        | LED alarm conditions          | 1     | –       | R/W   | 0 ~ 254     | –     | Each bit enables/disables LED flashing for a given alarm (see [3.4](#34-led-alarm-conditions))                                                     |
| 21          | 0x15        | Position loop P gain          | 1     | –       | R/W   | 0 ~ 254     | –     | Proportional gain for position control                                                                                                              |
| 22          | 0x16        | Position loop D gain          | 1     | –       | R/W   | 0 ~ 254     | –     | Derivative gain for position control                                                                                                                |
| 23          | 0x17        | Undefined                     | 1     | –       | R/W   | –           | –     | –                                                                                                                                                   |
| 24          | 0x18        | Minimum startup torque        | 1     | –       | R/W   | 0 ~ 254     | 0.1%  | Minimum torque output required to start movement                                                                                                    |
| 25          | 0x19        | Undefined                     | 1     | –       | R/W   | –           | –     | –                                                                                                                                                   |
| 26          | 0x1A        | Forward deadband              | 1     | 1       | R/W   | 0 ~ 16      | steps | Smallest unit is one minimum resolution angle                                                                                                       |
| 27          | 0x1B        | Reverse deadband              | 1     | 1       | R/W   | 0 ~ 16      | steps | Smallest unit is one minimum resolution angle                                                                                                       |
| 28 ~ 36     | 0x1C ~ 0x24 | Undefined                     | 1     | –       | R/W   | –           | –     | –                                                                                                                                                   |
| 37          | 0x25        | Holding torque                | 1     | 20      | R/W   | 0 ~ 254     | 1%    | Torque output after overload protection is triggered; e.g. 20 = 20% of max torque                                                                  |
| 38          | 0x26        | Protection time               | 1     | 200     | R/W   | 0 ~ 254     | 10 ms | Time that load exceeds overload torque before protection triggers; 200 = 2 s, max ≈ 2.5 s                                                          |
| 39          | 0x24        | Overload torque               | 1     | 80      | R/W   | 0 ~ 254     | 1%    | Threshold torque for starting the overload protection timer; 80 = 80% of max torque                                                                 |

---

### 2.3 SRAM Control

| Address DEC | Address HEX | Function Name   | Bytes | Default | Access | Range                | Unit   | Description                                                                                                                                                            |
|-------------|-------------|-----------------|-------|---------|--------|----------------------|--------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 40          | 0x28        | Torque switch   | 1     | 0       | R/W   | 0 ~ 2                | –      | 0: torque off / free; 1: torque on; 2: damping mode                                                                                                                   |
| 41          | 0x29        | Undefined       | 1     | –       | R/W   | –                    | –      | –                                                                                                                                                                      |
| 42          | 0x2A        | Goal position   | 2     | 0       | R/W   | 0 ~ 1023             | steps  | Each step is one minimum resolution angle; absolute position control. Max value corresponds to maximum effective angle                                                |
| 44          | 0x2C        | Run time        | 2     | 0       | R/W   | 0 ~ 9999 / -1000~1000| 1 ms / 0.1% | Time from current position to goal position when **run speed = 0**. In motor mode, this sets output PWM duty; bit 10 is direction bit                                  |
| 46          | 0x2E        | Run speed       | 2     | Factory default max speed | R/W | 0 ~ 1000           | steps/s| Steps per second (movement speed)                                                                                                                                     |
| 48          | 0x30        | Lock flag       | 1     | 1       | R/W   | 0 ~ 1                | –      | 0: unlock EPROM write, values written to EPROM addresses are stored after power-off; 1: lock EPROM write, values written to EPROM addresses are **not** stored        |
| 49 ~ 56     | 0x32~0x36   | Undefined       | 1     |         |        |                      |        | –                                                                                                                                                                      |

---

### 2.4 SRAM Feedback

| Address DEC | Address HEX | Function Name     | Bytes | Default | Access | Range | Unit   | Description                                                                                                                                                        |
|-------------|-------------|-------------------|-------|---------|--------|-------|--------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 56          | 0x38        | Current position  | 2     | –       | R      | –     | steps  | Current position in steps; each step is one minimum resolution angle. Absolute position mode; max value corresponds to maximum effective angle                    |
| 58          | 0x3A        | Current speed     | 2     | –       | R      | –     | steps/s| Current motor speed in steps per second                                                                                                                            |
| 60          | 0x3C        | Current load      | 2     | –       | R      | –     | 0.1%   | Current control output duty cycle driving the motor; bit 10 is direction bit                                                                                       |
| 62          | 0x3E        | Current voltage   | 1     | –       | R      | –     | 0.1 V  | Current servo supply voltage                                                                                                                                       |
| 63          | 0x3F        | Current temperature | 1   | –       | R      | –     | °C     | Current internal servo temperature                                                                                                                                 |
| 64          | 0x40        | Async write flag  | 1     | 0       | R      | –     | –      | Flag used when asynchronous write commands are used                                                                                                               |
| 65          | 0x41        | Servo status      | 1     | 0       | R      | –     | –      | Bits set to 1 indicate corresponding error(s) (see [3.2](#32-servo-status))                                                                                       |
| 66          | 0x42        | Movement flag     | 1     | 0       | R      | –     | –      | 1 while servo is moving; 0 when it has reached the target and stopped; stays 0 if no new goal position is given                                                   |

---

### 2.5 Factory Parameters

| Address DEC | Address HEX | Function Name                  | Bytes | Default | Access | Range | Unit | Description |
|-------------|-------------|--------------------------------|-------|---------|--------|-------|------|-------------|
| 78          | 0x4E        | Max step in PWM mode          | 1     | 20      | R      | –     | –    | –           |
| 79          | 0x50        | Movement speed threshold × 50 | 1     | 1       | R      | –     | –    | –           |
| 80          | 0x51        | DTs (ms)                      | 1     | 20      | R      | –     | –    | –           |
| 81          | 0x52        | Min speed limit × 50          | 1     | 1       | R      | –     | –    | –           |
| 82          | 0x53        | Max speed limit × 50          | 1     | –       | R      | –     | –    | –           |
| 83          | 0x54        | Acceleration                  | 1     | 20      | R      | –     | –    | –           |

---

## 3. Special Byte Description

---

### 3.1 Servo Phase

**Bits / weight: description**

- **BIT0 (1)**: Drive direction phase  
  - 0: normal direction  
  - 1: reversed direction
- **BIT1 (2)**: –––
- **BIT2 (4)**: –––
- **BIT3 (8)**: Speed mode  
  - 0: speed = 0 means stop  
  - 1: speed = 0 means maximum speed
- **BIT4 (16)**: –––
- **BIT5 (32)**: PWM phase  
  - 0: in-phase  
  - 1: inverted
- **BIT6 (64)**: Voltage mode  
  - 0: 1.5 k low-voltage sensing  
  - 1: 1 k high-voltage sensing
- **BIT7 (128)**: –––

> If multiple bits are set at the same time, the **phase value** is the **sum** of the values of all set bits.

---

### 3.2 Servo Status

**Servo status: 0 = normal, 1 = fault**

**Bits / weight: description**

- **BIT0 (1)**: Voltage status  
- **BIT1 (2)**: –––  
- **BIT2 (4)**: Temperature status  
- **BIT3 (8)**: –––  
- **BIT4 (16)**: –––  
- **BIT5 (32)**: Load status  
- **BIT6 (64)**: –––  
- **BIT7 (128)**: –––  

> If multiple fault conditions are present, the **status value** is the **sum** of the corresponding bit values.  
> Example: over/under-voltage and over-temperature → status = 4 + 1 = **5**.

---

### 3.3 Unload Conditions

**Unload conditions: 0 = disabled, 1 = enabled**  
(“Unload” = torque is turned off as protection.)

**Bits / weight: description**

- **BIT0 (1)**: Voltage protection  
- **BIT1 (2)**: –––  
- **BIT2 (4)**: Over-temperature protection  
- **BIT3 (8)**: –––  
- **BIT4 (16)**: –––  
- **BIT5 (32)**: Overload protection  
- **BIT6 (64)**: –––  
- **BIT7 (128)**: –––  

> If multiple bits are set, the **unload condition value** is the **sum** of the bit values.  
> Example: voltage protection + over-temperature protection enabled → unload value = 4 + 1 = **5**.

---

### 3.4 LED Alarm Conditions

**LED alarm conditions: 0 = off, 1 = on**

**Bits / weight: description**

- **BIT0 (1)**: Voltage alarm  
- **BIT1 (2)**: –––  
- **BIT2 (4)**: Over-temperature alarm  
- **BIT3 (8)**: –––  
- **BIT4 (16)**: –––  
- **BIT5 (32)**: Overload alarm  
- **BIT6 (64)**: –––  
- **BIT7 (128)**: –––  

> If multiple bits are set, the **LED alarm condition value** is the **sum** of the bit values.  
> Example: voltage alarm + over-temperature alarm enabled → alarm value = 4 + 1 = **5**.
