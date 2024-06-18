# Lid_control
Control the motor to open and close the lid.
![image](https://github.com/WanL0q/Lid_control/assets/134664967/617fd9eb-666a-43de-a6de-06471e5be1a0)

# CAN Frames (Device ID: 0x1A)
## Lid Command Frame

| **ID package**          | **Cycle(ms)** | **Receive-timeout(ms)** | **Data length**                                                             |
|:-----------------------:|:-------------:|:-----------------------:|:---------------------------------------------------------------------------:|
| 0x01                    | 100ms         | 500ms                   | 0x08                                                                        |
| **Location**            |  **Function** | **Data type**           | **Description**                                                             |
| byte[0]                 | lid_cmd       | unsigned int8            | 0: close<br>1: open<br>                                                     |
| byte[1]                 | Reserve       | _                       | 0x00                                                                        |
| byte[2]                 | Reserve       | _                       | 0x00                                                                        |
| byte[3]                 | Reserve       | _                       | 0x00                                                                        |
| byte[4]                 | Reserve       | _                       | 0x00                                                                        |
| byte[5]                 | Reserve       | _                       | 0x00                                                                        |
| byte[6]                 | Reserve       | _                       | 0x00                                                                        |
| byte[7]                 | Reserve       | _                       | 0x00                                                                        |
## Lid Configuration Frame

| **ID package**         | **Cycle(ms)** | **Receive-timeout(ms)** | **Data length** |
|:----------------------:|:-------------:|:-----------------------:|:---------------:|
| 0x02                   | -             | -                       | 0x08            |
| **Location**           | **Function**  | **Data type**           | **Description** |
| byte[0]                |Close spped    | unsigned int8           | Range [0,255]   |
| byte[1]                |Open speed     | unsigned int8           | Range [0,255]   |
| byte[2]                |Threshold upper 8 bit | unsigned int16   | [0,255]         |
| byte[3]                |Threshold lower 8 bit |                  | [0,255]         |
| byte[4]                |Reserve        | _                       | 0x00            |
| byte[5]                |Reserve        | _                       | 0x00            |
| byte[6]                |Reserve        | _                       | 0x00            |
| byte[7]                |Reserve        | _                       | 0x00            |

## Lid Status Feedback Frame
### Frame 1
| **ID package**         | **Cycle(ms)** | **Receive-timeout(ms)** | **Data length** |
|:----------------------:|:-------------:|:-----------------------:|:---------------:|
| 0x11                   | 100ms         | -                       | 0x08            |
| **Location**           | **Function**  | **Data type**           | **Description** |
| byte[0]                | Status        | unsigned int8           | 0: opend<br>1: running<br>2: closed<br>3: initializing<br>4: stuck |
| byte[1]<br>byte[2]     |Threshold upper 8 bit<br>Threshold lower 8 bit| unsigned int16    | Current threshold (mA)|                  |                       |
| byte[3]                |               | bit[0]<br>bit[1]<br>bit[2]<br>bit[3]<br>bit[4]<br>bit[5]<br>bit[6]<br>bit[7]| Limit switch 1<br>Limit switch 2<br>Lock switch<br>Lid close BT<br>Reserve<br>Reserve<br>Reserve<br>Reserve |
| byte[4]<br>byte[5]     |Current upper 8 bit<br>Current lower 8 bit| unsigned int16    | Current (mA)|                  |                       |
| byte[4]<br>byte[5]     |Voltage upper 8 bit<br>Voltage lower 8 bit| unsigned int16    | Voltage (0.1V)|                  |                       |
### Frame 2
| **ID package**         | **Cycle(ms)** | **Receive-timeout(ms)** | **Data length** |
|:----------------------:|:-------------:|:-----------------------:|:---------------:|
| 0x12                   | 100ms         | -                       | 0x08            |
| **Location**           | **Function**  | **Data type**           | **Description** |
| byte[0]                |error | bit[0]<br>bit[1]<br>bit[2]<br>bit[3]<br>bit[4]<br>bit[5]<br>bit[6]<br>bit[7]| Communication current sensor failure (0: Normal, 1: Failure)<br> EEPROM failure (0: Normal, 1: Failure)<br>Reserve<br>Reserve<br>Reserve<br>Reserve<br>Reserve<br>Reserve<br>|
| byte[1]                |Reserve        | -                       | 0x00            |
| byte[2]                |Reserve        | -                       | 0x00            |
| byte[3]                |Reserve        | -                       | 0x00            |
| byte[4]                |Reserve        | -                       | 0x00            |
| byte[5]                |Reserve        | -                       | 0x00            |
| byte[6]                |Reserve        | -                       | 0x00            |
| byte[7]                |Reserve        | -                       | 0x00            |