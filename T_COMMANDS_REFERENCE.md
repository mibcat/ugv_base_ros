# T Commands Reference

Complete overview of all T (command type) commands used in the UGV ROS Driver project.

## Emergency Control

| T   | Command Name        | Description          |
| --- | ------------------- | -------------------- |
| 0   | CMD_EMERGENCY_STOP  | Emergency stop       |
| 999 | CMD_RESET_EMERGENCY | Reset emergency flag |

## UGV Base Control

| T   | Command Name                  | Description                                              |
| --- | ----------------------------- | -------------------------------------------------------- |
| 1   | CMD_SPEED_CTRL                | Speed input control (L/R motors)                         |
| 11  | CMD_PWM_INPUT                 | PWM input control (±255)                                 |
| 13  | CMD_ROS_CTRL                  | ROS control (X velocity, Z angular velocity)             |
| 2   | CMD_SET_MOTOR_PID             | Motor PID & windup limits configuration                  |
| 3   | CMD_OLED_CTRL                 | OLED display control                                     |
| -3  | CMD_OLED_DEFAULT              | OLED default display                                     |
| 4   | CMD_MODULE_TYPE               | Module type settings (0=nothing, 1=RoArm-M2-S, 2=Gimbal) |
| 90  | CMD_SET_EFFECTIVE_TRACK_WIDTH | Set effective track width (w)                            |
| 91  | CMD_GET_EFFECTIVE_TRACK_WIDTH | Get effective track width                                |

## IMU & Feedback Control

| T   | Command Name               | Description                                                  |
| --- | -------------------------- | ------------------------------------------------------------ |
| 126 | CMD_GET_IMU_DATA           | Get IMU data                                                 |
| 127 | CMD_CALI_IMU_STEP          | Calibrate IMU (5 seconds, robot must be on ground and still) |
| 128 | CMD_GET_IMU_OFFSET         | Get IMU offset                                               |
| 129 | CMD_SET_IMU_OFFSET         | Set IMU offset                                               |
| 130 | CMD_BASE_FEEDBACK          | Base feedback request                                        |
| 131 | CMD_BASE_FEEDBACK_FLOW     | Enable/disable base feedback flow (0=off, 1=on)              |
| 142 | CMD_FEEDBACK_FLOW_INTERVAL | Set extra delay time for feedback (ms)                       |
| 143 | CMD_UART_ECHO_MODE         | Set UART echo mode (0=off, 1=on)                             |

## Speed Control

| T   | Command Name      | Description          |
| --- | ----------------- | -------------------- |
| 138 | CMD_SET_SPD_RATE  | Set speed rate (L/R) |
| 139 | CMD_GET_SPD_RATE  | Get speed rate       |
| 140 | CMD_SAVE_SPD_RATE | Save speed rate      |

## Light & Gimbal Control

| T   | Command Name           | Description                         |
| --- | ---------------------- | ----------------------------------- |
| 132 | CMD_LED_CTRL           | LED control (IO4/IO5)               |
| 133 | CMD_GIMBAL_CTRL_SIMPLE | Simple gimbal control (X, Y angles) |
| 134 | CMD_GIMBAL_CTRL_MOVE   | Gimbal move control (X, Y, SX, SY)  |
| 135 | CMD_GIMBAL_CTRL_STOP   | Gimbal stop                         |
| 136 | CMD_HEART_BEAT_SET     | Change heart beat delay (ms)        |
| 137 | CMD_GIMBAL_STEADY      | Gimbal stabilization (on/off)       |
| 141 | CMD_GIMBAL_USER_CTRL   | Gimbal user control (X, Y, SPD)     |

## EoAT (End of Arm Tool) Configuration

| T   | Command Name    | Description                                    |
| --- | --------------- | ---------------------------------------------- |
| 124 | CMD_EOAT_TYPE   | EoAT type settings (0=gripper, 1=wrist)        |
| 125 | CMD_CONFIG_EOAT | EoAT assemble configuration (position, ea, eb) |

## Arm Control - Basic

| T   | Command Name          | Description                                             |
| --- | --------------------- | ------------------------------------------------------- |
| 100 | CMD_MOVE_INIT         | Move to init position (no interpolation)                |
| 101 | CMD_SINGLE_JOINT_CTRL | Single joint control (joint, rad, spd, acc)             |
| 102 | CMD_JOINTS_RAD_CTRL   | All joints radian control (base, shoulder, elbow, hand) |
| 103 | CMD_SINGLE_AXIS_CTRL  | Single axis control (axis: 1=x, 2=y, 3=z, 4=t)          |
| 115 | CMD_SWITCH_OFF        | Switch off arm                                          |

## Arm Control - Position

| T    | Command Name           | Description                                    |
| ---- | ---------------------- | ---------------------------------------------- |
| 104  | CMD_XYZT_GOAL_CTRL     | XYZT goal control with speed (x, y, z, t, spd) |
| 1041 | CMD_XYZT_DIRECT_CTRL   | XYZT direct control (without speed)            |
| 105  | CMD_SERVO_RAD_FEEDBACK | Servo feedback in radians                      |
| 110  | CMD_SET_NEW_X          | Set new X-axis (xAxisAngle)                    |

## Arm Control - Hand/EoAT

| T   | Command Name         | Description                                      |
| --- | -------------------- | ------------------------------------------------ |
| 106 | CMD_EOAT_HAND_CTRL   | Hand joint control (release, grab, custom angle) |
| 107 | CMD_EOAT_GRAB_TORQUE | Set grab torque                                  |

## Arm Control - Advanced

| T   | Command Name           | Description                                          |
| --- | ---------------------- | ---------------------------------------------------- |
| 108 | CMD_SET_JOINT_PID      | Set joint P/I values                                 |
| 109 | CMD_RESET_PID          | Reset PID values                                     |
| 111 | CMD_DELAY_MILLIS       | Set delay time (ms)                                  |
| 112 | CMD_DYNAMIC_ADAPTATION | Dynamic external force adaptation (mode, b, s, e, h) |
| 121 | CMD_SINGLE_JOINT_ANGLE | Single joint angle control (degree, spd, acc)        |
| 122 | CMD_JOINTS_ANGLE_CTRL  | All joints angle control (degree, spd, acc)          |
| 123 | CMD_CONSTANT_CTRL      | Constant control (angle/xyzt mode, axis, cmd, spd)   |
| 144 | CMD_ARM_CTRL_UI        | Arm control UI (E, Z, R)                             |

## File Control

| T   | Command Name     | Description                         |
| --- | ---------------- | ----------------------------------- |
| 200 | CMD_SCAN_FILES   | Scan files in flash                 |
| 201 | CMD_CREATE_FILE  | Create new file with content        |
| 202 | CMD_READ_FILE    | Get file content                    |
| 203 | CMD_DELETE_FILE  | Remove file from flash              |
| 204 | CMD_APPEND_LINE  | Add line at end of file             |
| 205 | CMD_INSERT_LINE  | Insert line at specific line number |
| 206 | CMD_REPLACE_LINE | Replace specific line in file       |
| 207 | CMD_READ_LINE    | Read specific line from file        |
| 208 | CMD_DELETE_LINE  | Delete specific line from file      |
| 210 | CMD_TORQUE_CTRL  | Torque lock control (on/off)        |

## Mission Control

| T   | Command Name          | Description                             |
| --- | --------------------- | --------------------------------------- |
| 220 | CMD_CREATE_MISSION    | Create mission in flash                 |
| 221 | CMD_MISSION_CONTENT   | Get mission content                     |
| 222 | CMD_APPEND_STEP_JSON  | Append step using JSON input            |
| 223 | CMD_APPEND_STEP_FB    | Append step using feedback              |
| 224 | CMD_APPEND_DELAY      | Append delay (ms) to mission            |
| 225 | CMD_INSERT_STEP_JSON  | Insert step at position using JSON      |
| 226 | CMD_INSERT_STEP_FB    | Insert step at position using feedback  |
| 227 | CMD_INSERT_DELAY      | Insert delay at step position           |
| 228 | CMD_REPLACE_STEP_JSON | Replace step using JSON input           |
| 229 | CMD_REPLACE_STEP_FB   | Replace step using feedback             |
| 230 | CMD_REPLACE_DELAY     | Replace step with delay                 |
| 231 | CMD_DELETE_STEP       | Delete step from mission                |
| 241 | CMD_MOVE_TO_STEP      | Move to specific step in mission        |
| 242 | CMD_MISSION_PLAY      | Play mission (repeatTimes: -1=infinite) |

## ESP-NOW Settings

| T   | Command Name                | Description                                                                         |
| --- | --------------------------- | ----------------------------------------------------------------------------------- |
| 300 | CMD_BROADCAST_FOLLOWER      | Broadcast follower mode (mode: 0=custom, 1=broadcast)                               |
| 301 | CMD_ESP_NOW_CONFIG          | ESP-NOW mode config (0=none, 1=flow-leader-group, 2=flow-leader-single, 3=follower) |
| 302 | CMD_GET_MAC_ADDRESS         | Get device MAC address                                                              |
| 303 | CMD_ESP_NOW_ADD_FOLLOWER    | Add follower MAC to peer                                                            |
| 304 | CMD_ESP_NOW_REMOVE_FOLLOWER | Remove follower from peer                                                           |
| 305 | CMD_ESP_NOW_GROUP_CTRL      | Send command to group of devices                                                    |
| 306 | CMD_ESP_NOW_SINGLE          | Send command to single device or broadcast                                          |

## WiFi Settings

| T   | Command Name                     | Description                                             |
| --- | -------------------------------- | ------------------------------------------------------- |
| 401 | CMD_WIFI_ON_BOOT                 | Config WiFi mode on boot (0=off, 1=ap, 2=sta, 3=ap+sta) |
| 402 | CMD_SET_AP                       | Config AP mode (ssid, password)                         |
| 403 | CMD_SET_STA                      | Config STA mode (ssid, password)                        |
| 404 | CMD_WIFI_APSTA                   | Config AP/STA mode                                      |
| 405 | CMD_WIFI_INFO                    | Get WiFi info                                           |
| 406 | CMD_WIFI_CONFIG_CREATE_BY_STATUS | Create wifiConfig.json from current status              |
| 407 | CMD_WIFI_CONFIG_CREATE_BY_INPUT  | Create wifiConfig.json from input                       |
| 408 | CMD_WIFI_STOP                    | Disconnect WiFi                                         |

## Servo Settings

| T   | Command Name      | Description                             |
| --- | ----------------- | --------------------------------------- |
| 501 | CMD_SET_SERVO_ID  | Change servo ID                         |
| 502 | CMD_SET_MIDDLE    | Set current position as middle position |
| 503 | CMD_SET_SERVO_PID | Set P/PID of single servo               |

## ESP32 System Settings

| T   | Command Name           | Description                                           |
| --- | ---------------------- | ----------------------------------------------------- |
| 600 | CMD_REBOOT             | Reboot device                                         |
| 601 | CMD_FREE_FLASH_SPACE   | Get free flash space size                             |
| 602 | CMD_BOOT_MISSION_INFO  | Get boot mission info                                 |
| 603 | CMD_RESET_BOOT_MISSION | Reset boot mission                                    |
| 604 | CMD_NVS_CLEAR          | Clear NVS (for WiFi issues)                           |
| 605 | CMD_INFO_PRINT         | Set info print mode (0=off, 1=debug, 2=flow feedback) |

## System & Device Type

| T   | Command Name    | Description                   |
| --- | --------------- | ----------------------------- |
| 900 | CMD_MM_TYPE_SET | Set main type and module type |

## Feedback Messages

| T    | Message Type        | Description                                                    |
| ---- | ------------------- | -------------------------------------------------------------- |
| 1001 | FEEDBACK_BASE_INFO  | Base info feedback (L, R, gyro, accel, mag, odometry, voltage) |
| 1002 | FEEDBACK_IMU_DATA   | IMU data feedback (gx, gy, gz, ax, ay, az, mx, my, mz)         |
| 1004 | CMD_ESP_NOW_SEND    | ESP-NOW send status                                            |
| 1005 | CMD_BUS_SERVO_ERROR | Bus servo error feedback                                       |

## All Commands Sorted by Number

| T    | Command Name                     | Category                | JSON Example                                                                        |
| ---- | -------------------------------- | ----------------------- | ----------------------------------------------------------------------------------- |
| -3   | CMD_OLED_DEFAULT                 | UGV Base Control        | `{"T":-3}`                                                                          |
| 0    | CMD_EMERGENCY_STOP               | Emergency Control       | `{"T":0}`                                                                           |
| 1    | CMD_SPEED_CTRL                   | UGV Base Control        | `{"T":1,"L":0.5,"R":0.5}`                                                           |
| 2    | CMD_SET_MOTOR_PID                | UGV Base Control        | `{"T":2,"P":200,"I":2500,"D":0,"L":255}`                                            |
| 3    | CMD_OLED_CTRL                    | UGV Base Control        | `{"T":3,"lineNum":0,"Text":"putYourTextHere"}`                                      |
| 4    | CMD_MODULE_TYPE                  | UGV Base Control        | `{"T":4,"cmd":0}`                                                                   |
| 11   | CMD_PWM_INPUT                    | UGV Base Control        | `{"T":11,"L":164,"R":164}`                                                          |
| 13   | CMD_ROS_CTRL                     | UGV Base Control        | `{"T":13,"X":0.1,"Z":0.3}`                                                          |
| 90   | CMD_SET_EFFECTIVE_TRACK_WIDTH    | UGV Base Control        | `{"T":90,"w":0.200}`                                                                |
| 91   | CMD_GET_EFFECTIVE_TRACK_WIDTH    | UGV Base Control        | `{"T":91}`                                                                          |
| 100  | CMD_MOVE_INIT                    | Arm Control - Basic     | `{"T":100}`                                                                         |
| 101  | CMD_SINGLE_JOINT_CTRL            | Arm Control - Basic     | `{"T":101,"joint":0,"rad":0,"spd":0,"acc":10}`                                      |
| 102  | CMD_JOINTS_RAD_CTRL              | Arm Control - Basic     | `{"T":102,"base":0,"shoulder":0,"elbow":1.57,"hand":1.57,"spd":0,"acc":10}`         |
| 103  | CMD_SINGLE_AXIS_CTRL             | Arm Control - Basic     | `{"T":103,"axis":2,"pos":0,"spd":0.25}`                                             |
| 104  | CMD_XYZT_GOAL_CTRL               | Arm Control - Position  | `{"T":104,"x":235,"y":0,"z":234,"t":3.14,"spd":0.25}`                               |
| 105  | CMD_SERVO_RAD_FEEDBACK           | Arm Control - Position  | Feedback only                                                                       |
| 106  | CMD_EOAT_HAND_CTRL               | Arm Control - Hand/EoAT | `{"T":106,"cmd":1.57,"spd":0,"acc":0}`                                              |
| 107  | CMD_EOAT_GRAB_TORQUE             | Arm Control - Hand/EoAT | `{"T":107,"tor":200}`                                                               |
| 108  | CMD_SET_JOINT_PID                | Arm Control - Advanced  | `{"T":108,"joint":3,"p":16,"i":0}`                                                  |
| 109  | CMD_RESET_PID                    | Arm Control - Advanced  | `{"T":109}`                                                                         |
| 110  | CMD_SET_NEW_X                    | Arm Control - Position  | `{"T":110,"xAxisAngle":0}`                                                          |
| 111  | CMD_DELAY_MILLIS                 | Arm Control - Advanced  | `{"T":111,"cmd":3000}`                                                              |
| 112  | CMD_DYNAMIC_ADAPTATION           | Arm Control - Advanced  | `{"T":112,"mode":1,"b":60,"s":110,"e":50,"h":50}`                                   |
| 115  | CMD_SWITCH_OFF                   | Arm Control - Basic     | `{"T":115}`                                                                         |
| 121  | CMD_SINGLE_JOINT_ANGLE           | Arm Control - Advanced  | `{"T":121,"joint":1,"angle":0,"spd":10,"acc":10}`                                   |
| 122  | CMD_JOINTS_ANGLE_CTRL            | Arm Control - Advanced  | `{"T":122,"b":0,"s":0,"e":90,"h":180,"spd":10,"acc":10}`                            |
| 123  | CMD_CONSTANT_CTRL                | Arm Control - Advanced  | `{"T":123,"m":0,"axis":0,"cmd":0,"spd":3}`                                          |
| 124  | CMD_EOAT_TYPE                    | EoAT Configuration      | `{"T":124,"mode":0}`                                                                |
| 125  | CMD_CONFIG_EOAT                  | EoAT Configuration      | `{"T":125,"pos":3,"ea":0,"eb":20}`                                                  |
| 126  | CMD_GET_IMU_DATA                 | IMU & Feedback Control  | `{"T":126}`                                                                         |
| 127  | CMD_CALI_IMU_STEP                | IMU & Feedback Control  | `{"T":127}`                                                                         |
| 128  | CMD_GET_IMU_OFFSET               | IMU & Feedback Control  | `{"T":128}`                                                                         |
| 129  | CMD_SET_IMU_OFFSET               | IMU & Feedback Control  | `{"T":129,"gx":0,"gy":0,"gz":0,"ax":0,"ay":0,"az":0,"cx":0,"cy":0,"cz":0}`          |
| 130  | CMD_BASE_FEEDBACK                | IMU & Feedback Control  | `{"T":130}`                                                                         |
| 131  | CMD_BASE_FEEDBACK_FLOW           | IMU & Feedback Control  | `{"T":131,"cmd":0}`                                                                 |
| 132  | CMD_LED_CTRL                     | Light & Gimbal Control  | `{"T":132,"IO4":255,"IO5":255}`                                                     |
| 133  | CMD_GIMBAL_CTRL_SIMPLE           | Light & Gimbal Control  | `{"T":133,"X":45,"Y":45,"SPD":0,"ACC":0}`                                           |
| 134  | CMD_GIMBAL_CTRL_MOVE             | Light & Gimbal Control  | `{"T":134,"X":45,"Y":45,"SX":300,"SY":300}`                                         |
| 135  | CMD_GIMBAL_CTRL_STOP             | Light & Gimbal Control  | `{"T":135}`                                                                         |
| 136  | CMD_HEART_BEAT_SET               | Light & Gimbal Control  | `{"T":136,"cmd":3000}`                                                              |
| 137  | CMD_GIMBAL_STEADY                | Light & Gimbal Control  | `{"T":137,"s":0,"y":0}`                                                             |
| 138  | CMD_SET_SPD_RATE                 | Speed Control           | `{"T":138,"L":1,"R":1}`                                                             |
| 139  | CMD_GET_SPD_RATE                 | Speed Control           | `{"T":139}`                                                                         |
| 140  | CMD_SAVE_SPD_RATE                | Speed Control           | `{"T":140}`                                                                         |
| 141  | CMD_GIMBAL_USER_CTRL             | Light & Gimbal Control  | `{"T":141,"X":0,"Y":0,"SPD":300}`                                                   |
| 142  | CMD_FEEDBACK_FLOW_INTERVAL       | IMU & Feedback Control  | `{"T":142,"cmd":0}`                                                                 |
| 143  | CMD_UART_ECHO_MODE               | IMU & Feedback Control  | `{"T":143,"cmd":0}`                                                                 |
| 144  | CMD_ARM_CTRL_UI                  | Arm Control - Advanced  | `{"T":144,"E":100,"Z":0,"R":0}`                                                     |
| 200  | CMD_SCAN_FILES                   | File Control            | `{"T":200}`                                                                         |
| 201  | CMD_CREATE_FILE                  | File Control            | `{"T":201,"name":"file.txt","content":"inputContentHere."}`                         |
| 202  | CMD_READ_FILE                    | File Control            | `{"T":202,"name":"file.txt"}`                                                       |
| 203  | CMD_DELETE_FILE                  | File Control            | `{"T":203,"name":"file.txt"}`                                                       |
| 204  | CMD_APPEND_LINE                  | File Control            | `{"T":204,"name":"file.txt","content":"inputContentHere."}`                         |
| 205  | CMD_INSERT_LINE                  | File Control            | `{"T":205,"name":"file.txt","lineNum":3,"content":"content"}`                       |
| 206  | CMD_REPLACE_LINE                 | File Control            | `{"T":206,"name":"file.txt","lineNum":3,"content":"Content"}`                       |
| 207  | CMD_READ_LINE                    | File Control            | `{"T":207,"name":"file.txt","lineNum":3}`                                           |
| 208  | CMD_DELETE_LINE                  | File Control            | `{"T":208,"name":"file.txt","lineNum":3}`                                           |
| 210  | CMD_TORQUE_CTRL                  | File Control            | `{"T":210,"cmd":0}`                                                                 |
| 220  | CMD_CREATE_MISSION               | Mission Control         | `{"T":220,"name":"mission_a","intro":"test mission"}`                               |
| 221  | CMD_MISSION_CONTENT              | Mission Control         | `{"T":221,"name":"mission_a"}`                                                      |
| 222  | CMD_APPEND_STEP_JSON             | Mission Control         | `{"T":222,"name":"mission_a","step":"{...}"}`                                       |
| 223  | CMD_APPEND_STEP_FB               | Mission Control         | `{"T":223,"name":"mission_a","spd":0.25}`                                           |
| 224  | CMD_APPEND_DELAY                 | Mission Control         | `{"T":224,"name":"mission_a","delay":3000}`                                         |
| 225  | CMD_INSERT_STEP_JSON             | Mission Control         | `{"T":225,"name":"mission_a","stepNum":3,"step":"{...}"}`                           |
| 226  | CMD_INSERT_STEP_FB               | Mission Control         | `{"T":226,"name":"mission_a","stepNum":3,"spd":0.25}`                               |
| 227  | CMD_INSERT_DELAY                 | Mission Control         | `{"T":227,"stepNum":3,"delay":3000}`                                                |
| 228  | CMD_REPLACE_STEP_JSON            | Mission Control         | `{"T":228,"name":"mission_a","stepNum":3,"step":"{...}"}`                           |
| 229  | CMD_REPLACE_STEP_FB              | Mission Control         | `{"T":229,"name":"mission_a","stepNum":3,"spd":0.25}`                               |
| 230  | CMD_REPLACE_DELAY                | Mission Control         | `{"T":230,"name":"mission_a","stepNum":3,"delay":3000}`                             |
| 231  | CMD_DELETE_STEP                  | Mission Control         | `{"T":231,"name":"mission_a","stepNum":3}`                                          |
| 241  | CMD_MOVE_TO_STEP                 | Mission Control         | `{"T":241,"name":"mission_a","stepNum":3}`                                          |
| 242  | CMD_MISSION_PLAY                 | Mission Control         | `{"T":242,"name":"mission_a","times":3}`                                            |
| 300  | CMD_BROADCAST_FOLLOWER           | ESP-NOW Settings        | `{"T":300,"mode":1}`                                                                |
| 301  | CMD_ESP_NOW_CONFIG               | ESP-NOW Settings        | `{"T":301,"mode":3}`                                                                |
| 302  | CMD_GET_MAC_ADDRESS              | ESP-NOW Settings        | `{"T":302}`                                                                         |
| 303  | CMD_ESP_NOW_ADD_FOLLOWER         | ESP-NOW Settings        | `{"T":303,"mac":"FF:FF:FF:FF:FF:FF"}`                                               |
| 304  | CMD_ESP_NOW_REMOVE_FOLLOWER      | ESP-NOW Settings        | `{"T":304,"mac":"FF:FF:FF:FF:FF:FF"}`                                               |
| 305  | CMD_ESP_NOW_GROUP_CTRL           | ESP-NOW Settings        | `{"T":305,"dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}`           |
| 306  | CMD_ESP_NOW_SINGLE               | ESP-NOW Settings        | `{"T":306,"mac":"FF:FF:FF:FF:FF:FF","dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0}` |
| 401  | CMD_WIFI_ON_BOOT                 | WiFi Settings           | `{"T":401,"cmd":3}`                                                                 |
| 402  | CMD_SET_AP                       | WiFi Settings           | `{"T":402,"ssid":"RoArm-M2","password":"12345678"}`                                 |
| 403  | CMD_SET_STA                      | WiFi Settings           | `{"T":403,"ssid":"JSBZY-2.4G","password":"waveshare0755"}`                          |
| 404  | CMD_WIFI_APSTA                   | WiFi Settings           | `{"T":404,"ap_ssid":"RoArm-M2","ap_password":"12345678"...}`                        |
| 405  | CMD_WIFI_INFO                    | WiFi Settings           | `{"T":405}`                                                                         |
| 406  | CMD_WIFI_CONFIG_CREATE_BY_STATUS | WiFi Settings           | `{"T":406}`                                                                         |
| 407  | CMD_WIFI_CONFIG_CREATE_BY_INPUT  | WiFi Settings           | `{"T":407,"mode":3,"ap_ssid":"RoArm-M2"...}`                                        |
| 408  | CMD_WIFI_STOP                    | WiFi Settings           | `{"T":408}`                                                                         |
| 501  | CMD_SET_SERVO_ID                 | Servo Settings          | `{"T":501,"raw":1,"new":11}`                                                        |
| 502  | CMD_SET_MIDDLE                   | Servo Settings          | `{"T":502,"id":11}`                                                                 |
| 503  | CMD_SET_SERVO_PID                | Servo Settings          | `{"T":503,"id":14,"p":16}`                                                          |
| 600  | CMD_REBOOT                       | ESP32 System Settings   | `{"T":600}`                                                                         |
| 601  | CMD_FREE_FLASH_SPACE             | ESP32 System Settings   | `{"T":601}`                                                                         |
| 602  | CMD_BOOT_MISSION_INFO            | ESP32 System Settings   | `{"T":602}`                                                                         |
| 603  | CMD_RESET_BOOT_MISSION           | ESP32 System Settings   | `{"T":603}`                                                                         |
| 604  | CMD_NVS_CLEAR                    | ESP32 System Settings   | `{"T":604}`                                                                         |
| 605  | CMD_INFO_PRINT                   | ESP32 System Settings   | `{"T":605,"cmd":1}`                                                                 |
| 900  | CMD_MM_TYPE_SET                  | System & Device Type    | `{"T":900,"main":1,"module":0}`                                                     |
| 999  | CMD_RESET_EMERGENCY              | Emergency Control       | `{"T":999}`                                                                         |
| 1001 | FEEDBACK_BASE_INFO               | Feedback Messages       | `{"T":1001,"L":0,"R":0,"gx":0,"gy":0,"gz":0...}`                                    |
| 1002 | FEEDBACK_IMU_DATA                | Feedback Messages       | `{"T":1002,"gx":0,"gy":0,"gz":0,"ax":0,"ay":0,"az":0...}`                           |
| 1004 | CMD_ESP_NOW_SEND                 | Feedback Messages       | `{"T":1004,"mac":"FF:FF:FF:FF:FF:FF","status":1}`                                   |
| 1005 | CMD_BUS_SERVO_ERROR              | Feedback Messages       | `{"T":1005,"id":1,"status":1}`                                                      |
| 1041 | CMD_XYZT_DIRECT_CTRL             | Arm Control - Position  | `{"T":1041,"x":235,"y":0,"z":234,"t":3.14}`                                         |

## Notes

- Joint numbering: 1=BASE_JOINT, 2=SHOULDER_JOINT, 3=ELBOW_JOINT, 4=EOAT_JOINT
- Speed units: steps/s or angle/s depending on context
- Acceleration units: steps/s² or angle/s²  (max: 22.5)
- Angles can be in radians or degrees depending on command
- T:3 (FEEDBACK_IMU_OFFSET) message includes bias values (gx, gy, gz, ax, ay, az, cx, cy, cz)
- For missions with repeatTimes = -1, mission plays forever
