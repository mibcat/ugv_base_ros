#include <ArduinoJson.h>
StaticJsonDocument<256> jsonCmdReceive;
StaticJsonDocument<256> jsonInfoSend;
StaticJsonDocument<1024> jsonInfoHttp;

#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>
#include <INA219_WE.h>
#include <LittleFS.h>
#include <PID_v2.h>
#include <SCServo.h>
#include <SimpleKalmanFilter.h>
#if ENABLE_HTTP_SERVER
#include <WebServer.h>
#endif
#include <WiFi.h>
#include <esp_now.h>
#include <esp_system.h>
#include <nvs_flash.h>

// functions for battery info.
#include "battery_ctrl.h"

// config for ugv.
#include "ugv_config.h"

// functions for oled.
#include "oled_ctrl.h"

// functions for the leds of UGV.
#include "ugv_led_ctrl.h"

#if ENABLE_ROARM
// functions for RoArm-M2 ctrl.
#include "RoArm-M2_module.h"
#endif

#if ENABLE_GIMBAL
// functions for gimbal ctrl.
#include "gimbal_module.h"
#endif

// define json cmd.
#include "json_cmd.h"

// functions for IMU ctrl.
#include "IMU_ctrl.h"

// functions for movtion ctrl.
#include "movtion_module.h"

// functions for editing the files in flash.
#include "files_ctrl.h"

// advance functions for ugv ctrl.
#include "ugv_advance.h"

#if ENABLE_WIRELESS
// functions for wifi ctrl.
#include "wifi_ctrl.h"

// functions for esp-now.
#include "esp_now_ctrl.h"
#endif

// functions for uart json ctrl.
#include "uart_ctrl.h"

#if ENABLE_HTTP_SERVER
// functions for http & web server.
#include "http_server.h"
#endif

#if ENABLE_ROARM
void moduleType_RoArmM2() {
  unsigned long curr_time = millis();
  if (curr_time - prev_time >= 10) {
    constantHandle();
    prev_time = curr_time;
  }

  RoArmM2_getPosByServoFeedback();

  // esp-now flow ctrl as a flow-leader.
  switch (espNowMode) {
    case 1:
      espNowGroupDevsFlowCtrl();
      break;
    case 2:
      espNowSingleDevFlowCtrl();
      break;
  }

  if (InfoPrint == 2) {
    RoArmM2_infoFeedback();
  }
}
#endif

void setup() {
  // setup serial
  auto new_size = Serial.setTxBufferSize(512);
  Serial.begin(115200);
  while (!Serial) {
  }
  Serial.println("Serial started with Tx buffer size: " + String(new_size));
  
  // setup i2c for IMU
  Wire.begin(S_SDA, S_SCL, 400000);

  ina219_init();
  inaDataUpdate();

  // set mainType & moduleType.
  // mainType: 1.RaspRover, 2.UGV Rover, 3.UGV Beast
  // moduleType: 0.Null, 1.RoArm, 2.PT
  mm_settings(mainType, moduleType);

  init_oled();
  screenLine_0 = String("GIT:") + GIT_COMMIT_HASH;

  // setup IMU
  screenLine_1 = "IMU + cal";
  oled_update();
  if (InfoPrint == 1) {
    Serial.println("Initialize IMU...");
  }
  imu_init();

  // functions for the leds on ugv.
  led_pin_init();

  // init the littleFS funcs in files_ctrl.h
  screenLine_1 = "LittleFS";
  oled_update();
  if (InfoPrint == 1) {
    Serial.println("Initialize LittleFS for Flash files ctrl.");
  }
  initFS();

  // init the funcs in switch_module.h
  screenLine_1 = "12V-sw ctl";
  oled_update();
  if (InfoPrint == 1) {
    Serial.println("Initialize the pins used for 12V-switch ctrl.");
  }
  movtionPinInit();

  // servos power up
  screenLine_1 = "Pwr servos";
  oled_update();
  if (InfoPrint == 1) {
    Serial.println("Power up the servos.");
  }
  delay(500);

#if ENABLE_ROARM
  // init servo ctrl functions.
  screenLine_1 = "servo ctrl";
  oled_update();
  if (InfoPrint == 1) {
    Serial.println("ServoCtrl init UART2TTL...");
  }
  RoArmM2_servoInit();

  // check the status of the servos.
  screenLine_1 = "servo stat";
  oled_update();
  if (InfoPrint == 1) {
    Serial.println("Bus servos status check...");
  }
  RoArmM2_initCheck(false);

  if (InfoPrint == 1 && RoArmM2_initCheckSucceed) {
    Serial.println("All bus servos status checked.");
  }
  if (RoArmM2_initCheckSucceed) {
    screenLine_1 = "servo ok";
  } else {
    screenLine_1 = "servo:" + servoFeedback[BASE_SERVO_ID - 11].status +
                   servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].status +
                   servoFeedback[SHOULDER_DRIVEN_SERVO_ID - 11].status +
                   servoFeedback[ELBOW_SERVO_ID - 11].status +
                   servoFeedback[GRIPPER_SERVO_ID - 11].status;
  }
  screenLine_1 = "mov to ini";
  oled_update();
  RoArmM2_resetPID();
  RoArmM2_moveInit();

  screenLine_1 = "res torque";
  oled_update();
  if (InfoPrint == 1) {
    Serial.println("Reset joint torque to ST_TORQUE_MAX.");
  }
  RoArmM2_dynamicAdaptation(0, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX,
                            ST_TORQUE_MAX);
#endif

#if ENABLE_WIRELESS
  screenLine_1 = "WiFi";
  oled_update();
  if (InfoPrint == 1) {
    Serial.println("WiFi init.");
  }
  initWifi();
#endif

#if ENABLE_HTTP_SERVER
  screenLine_1 = "http & web";
  oled_update();
  if (InfoPrint == 1) {
    Serial.println("http & web init.");
  }
  initHttpWebServer();
#endif

#if ENABLE_WIRELESS
  screenLine_1 = "ESP-NOW";
  oled_update();
  if (InfoPrint == 1) {
    Serial.println("ESP-NOW init.");
  }
  initEspNow();

  getThisDevMacAddress();
#endif

  screenLine_0 = "UGV ready";
  screenLine_1 = "";
  oled_update();
  if (InfoPrint == 1) {
    Serial.println("UGV started.");
  }

  initEncoders();

  pidControllerInit();

  led_pwm_ctrl(0, 0);

  if (InfoPrint == 1) {
    Serial.println("Application initialization settings.");
  }
  createMission("boot", "these cmds run automatically at boot.");
  missionPlay("boot", 1);
}

void loop() {
  // update IMU
  updateIMUData();

  // handle serial json cmd
  serialCtrl();

#if ENABLE_HTTP_SERVER
  server.handleClient();
#endif

  // read and compute the info of joints.
  switch (moduleType) {
    case 1:
#if ENABLE_ROARM
      moduleType_RoArmM2();
#endif
      break;
    case 2:
#if ENABLE_GIMBAL
      getGimbalFeedback();
      gimbalSteady(steadyGoalY);
#endif
      break;
  }

  // recv esp-now json cmd.
  if (runNewJsonCmd) {
    jsonCmdReceiveHandler();
    jsonCmdReceive.clear();
    runNewJsonCmd = false;
  }

  auto now = micros();
  if (now - lastWheelSpeedMeasureTime > adaptiveWindowUs) {
    getWheelSpeeds();
    PidControllerCompute();
    lastWheelSpeedMeasureTime = now;
  }

  oledInfoUpdate();

  if (baseFeedbackFlow) {
    baseInfoFeedback();
  }

  heartBeatCtrl();
}
