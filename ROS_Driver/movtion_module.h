#include <math.h>

// ============ Adaptive Measurement Window Parameters ============

// 100ms - minimum window for fast response at high speeds
const unsigned long WINDOW_US_MIN = 100000;
// 500ms - maximum window for accuracy at low speeds
const unsigned long WINDOW_US_MAX = 500000;
// 0.3 m/s - speed threshold for using minimum window
const float SPEED_FOR_MIN_WINDOW = 0.30;
// 0.05 m/s - speed threshold for using maximum window
const float SPEED_FOR_MAX_WINDOW = 0.05;

// Current adaptive measurement window in microseconds
static unsigned long adaptiveWindowUs = 100000;

// ============ Wheel Speed Ramping Parameters ============

// acceleration when ramping up in [m/s²]
const float WHEEL_ACCEL_MAX = 0.4;
// deceleration when ramping down in [m/s²]
const float WHEEL_DECEL_MAX = 1.0;

// Ramped wheel speed setpoints (for smooth acceleration/deceleration)
static float setpointARamped = 0.0;
static float setpointBRamped = 0.0;
static unsigned long lastSetGoalSpeedTime = 0;

bool usePIDCompute = true;
float speedFactorA = 1.0;
float speedFactorB = 1.0;
bool heartbeatStopFlag = false;
static unsigned long lastWheelSpeedMeasureTime{};

void switchEmergencyStop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void movtionPinInit() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  ledcAttach(PWMA, MOTOR_CONTROL_FREQ, ANALOG_WRITE_BITS);
  ledcAttach(PWMB, MOTOR_CONTROL_FREQ, ANALOG_WRITE_BITS);

  switchEmergencyStop();
}

void setSpdRate(float inputL, float inputR) {
  inputL = abs(inputL);
  if (inputL > 1) {
    inputL = 1;
  }
  inputR = abs(inputR);
  if (inputR > 1) {
    inputR = 1;
  }
  speedFactorA = inputL;
  speedFactorB = inputR;
}

void getSpdRate() {
  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = CMD_GET_SPD_RATE;

  jsonInfoHttp["L"] = speedFactorA;
  jsonInfoHttp["R"] = speedFactorB;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}

// movtion parts.
// A-left, B-right

ESP32Encoder encoderA;
ESP32Encoder encoderB;

// time stamp of last speed calculation
static unsigned long lastSpeedTime = 0;

// current left speeds [m/s]
float speedGetA;
// current right speeds [m/s]
float speedGetB;

// m/inc
float plusesRate = static_cast<float>(M_PI) * WHEEL_D / ONE_CIRCLE_PLUSES;

void initEncoders() {
  encoderA.attachHalfQuad(AENCA, AENCB);
  encoderB.attachHalfQuad(BENCA, BENCB);
  encoderA.setCount(0);
  encoderB.setCount(0);
}

// Calculate adaptive measurement window based on current wheel speeds
// Uses linear interpolation between SPEED_FOR_MAX_WINDOW and
// SPEED_FOR_MIN_WINDOW
void calculateAdaptiveWindowUs() {
  // Calculate maximum absolute wheel speed from global speedGetA and speedGetB
  float maxAbsSpeed =
      (abs(speedGetA) > abs(speedGetB)) ? abs(speedGetA) : abs(speedGetB);

  if (maxAbsSpeed >= SPEED_FOR_MIN_WINDOW) {
    // High speed: use minimum window for faster feedback
    adaptiveWindowUs = WINDOW_US_MIN;
  } else if (maxAbsSpeed <= SPEED_FOR_MAX_WINDOW) {
    // Low speed: use maximum window for better accuracy
    adaptiveWindowUs = WINDOW_US_MAX;
  } else {
    // Medium speed: linear interpolation between min and max
    float normalized = (maxAbsSpeed - SPEED_FOR_MAX_WINDOW) /
                       (SPEED_FOR_MIN_WINDOW - SPEED_FOR_MAX_WINDOW);
    adaptiveWindowUs =
        (unsigned long)(WINDOW_US_MAX -
                        normalized * (WINDOW_US_MAX - WINDOW_US_MIN));
  }
}

void getWheelSpeeds() {
  // get current encoder pulses
  auto encoderPulsesA = encoderA.getCount();
  auto encoderPulsesB = encoderB.getCount();

  // get current time and calculate dt since last speed calculation
  auto currentTime = micros();
  auto dt = (float)(currentTime - lastSpeedTime) / 1000000;

  // Calculate new absolute odometry from encoder counts
  float odomLeft, odomRight;
  if (!SET_MOTOR_DIR) {
    odomLeft = (float)encoderPulsesA * plusesRate;
    odomRight = (float)encoderPulsesB * plusesRate;
  } else {
    odomLeft = -(float)encoderPulsesA * plusesRate;
    odomRight = -(float)encoderPulsesB * plusesRate;
  }

  // Derive speeds from odometry changes (using previous en_odom values)
  speedGetA = (odomLeft - en_odom_l) / dt;
  speedGetB = (odomRight - en_odom_r) / dt;

  // Update odometry for next iteration
  en_odom_l = odomLeft;
  en_odom_r = odomRight;
  lastSpeedTime = currentTime;

  // Update adaptive measurement window based on current wheel speeds
  calculateAdaptiveWindowUs();
}

// --- PID Controller ---

PID_v2 pidA(__kp, __ki, __kd, PID::Direct);
PID_v2 pidB(__kp, __ki, __kd, PID::Direct);

double outputA = 0;
double outputB = 0;
double setpointA = 0;
double setpointB = 0;

float setpointABuffer;
float setpointBBuffer;

void pidControllerInit() {
  pidA.Start(speedGetA, outputA, setpointA);
  pidA.SetOutputLimits(-255, 255);
  pidA.SetMode(PID::Automatic);

  pidB.Start(speedGetB, outputB, setpointB);
  pidB.SetOutputLimits(-255, 255);
  pidB.SetMode(PID::Automatic);
}

void leftCtrl(float pwmInputA) {
  int pwmIntA = round(pwmInputA);
  if (SET_MOTOR_DIR) {
    if (pwmIntA < 0) {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      ledcWrite(PWMA, abs(pwmIntA));
    } else {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      ledcWrite(PWMA, abs(pwmIntA));
    }
  } else {
    if (pwmIntA < 0) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      ledcWrite(PWMA, abs(pwmIntA));
    } else {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      ledcWrite(PWMA, abs(pwmIntA));
    }
  }
}

void rightCtrl(float pwmInputB) {
  int pwmIntB = round(pwmInputB);
  if (SET_MOTOR_DIR) {
    if (pwmIntB < 0) {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      ledcWrite(PWMB, abs(pwmIntB));
    } else {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      ledcWrite(PWMB, abs(pwmIntB));
    }
  } else {
    if (pwmIntB < 0) {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      ledcWrite(PWMB, abs(pwmIntB));
    } else {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      ledcWrite(PWMB, abs(pwmIntB));
    }
  }
}

void setGoalSpeed(float inputLeft, float inputRight) {
  if (inputLeft < -2.0 || inputLeft > 2.0) {
    return;
  }

  if (inputRight < -2.0 || inputRight > 2.0) {
    return;
  }

  // Calculate scaled target setpoints
  float targetA = inputLeft * speedFactorA;
  float targetB = inputRight * speedFactorB;

  // Calculate time since last setGoalSpeed call
  unsigned long now_us = micros();
  float dt = (now_us - lastSetGoalSpeedTime) / 1e6f;
  lastSetGoalSpeedTime = now_us;

  // Safety clamp on dt: cap at 100ms, allow very small dt for smooth ramp
  if (dt > 0.1f) dt = 0.1f;
  if (dt < 0.0001f) dt = 0.0001f;

  // Ramp left wheel speed (use different rates for acceleration vs.
  // deceleration)
  float deltaNeededA = targetA - setpointARamped;
  // Accelerating: delta and current setpoint have same sign (speeding up in
  // current direction) Braking: delta and current setpoint have different signs
  // (slowing down current direction)
  bool isAcceleratingA = (deltaNeededA * setpointARamped) > 0;
  float maxDeltaA =
      isAcceleratingA ? WHEEL_ACCEL_MAX * dt : WHEEL_DECEL_MAX * dt;
  setpointARamped += fminf(fmaxf(deltaNeededA, -maxDeltaA), maxDeltaA);

  // Ramp right wheel speed (use different rates for acceleration vs.
  // deceleration)
  float deltaNeededB = targetB - setpointBRamped;
  // Accelerating: delta and current setpoint have same sign (speeding up in
  // current direction) Braking: delta and current setpoint have different signs
  // (slowing down current direction)
  bool isAcceleratingB = (deltaNeededB * setpointBRamped) > 0;
  float maxDeltaB =
      isAcceleratingB ? WHEEL_ACCEL_MAX * dt : WHEEL_DECEL_MAX * dt;
  setpointBRamped += fminf(fmaxf(deltaNeededB, -maxDeltaB), maxDeltaB);

  // Update PID setpoints with ramped values
  setpointA = setpointARamped;
  setpointB = setpointBRamped;

  if (setpointA != setpointABuffer) {
    pidA.Setpoint(setpointA);
    setpointABuffer = setpointA;
  }

  if (setpointB != setpointBBuffer) {
    pidB.Setpoint(setpointB);
    setpointBBuffer = setpointB;
  }
}

void PidControllerCompute() {
  if (!usePIDCompute) {
    return;
  }

  // Left wheel
  outputA = pidA.Run(speedGetA);
  // prevent motor creep
  if (setpointA == 0 && speedGetA == 0) {
    outputA = 0;
  }
  leftCtrl(outputA);

  // Right wheel
  outputB = pidB.Run(speedGetB);
  // prevent motor creep
  if (setpointB == 0 && speedGetB == 0) {
    outputB = 0;
  }
  rightCtrl(outputB);
}

void setPID(float inputP, float inputI, float inputD, float inputLimits) {
  __kp = inputP;
  __ki = inputI;
  __kd = inputD;
  windup_limits = inputLimits;
  pidA.SetTunings(__kp, __ki, __kd);
  pidB.SetTunings(__kp, __ki, __kd);
}

void rosCtrl(float rosX, float rosZ) {
  // Convert kinematic commands to wheel speeds [m/s]! via differential drive
  float goalA = rosX - (rosZ * TRACK_WIDTH / 2.0);
  float goalB = rosX + (rosZ * TRACK_WIDTH / 2.0);

  // Pass to setGoalSpeed for ramping, scaling, and PID update
  setGoalSpeed(goalA, goalB);
}

void heartBeatCtrl() {
  if (currentTimeMillis - lastCmdRecvTime > HEART_BEAT_DELAY) {
    // Keep commanding zero speed every loop iteration to enable ramping to
    // complete. The ramping in setGoalSpeed() will gradually decelerate the
    // wheels
    setGoalSpeed(0, 0);
    heartbeatStopFlag = true;
  } else {
    // Reset flag when commands are being received again
    heartbeatStopFlag = false;
  }
}

void changeHeartBeatDelay(int inputCmd) { HEART_BEAT_DELAY = inputCmd; }

void mm_settings(byte inputMain, byte inputModule) {
  mainType = inputMain;
  moduleType = inputModule;

  // mainType:01 RaspRover
  // #define WHEEL_D 0.0800
  // #define ONE_CIRCLE_PLUSES  2100
  // #define TRACK_WIDTH  0.125
  // #define SET_MOTOR_DIR false

  // mainType:02 UGV Rover
  // #define WHEEL_D 0.0800
  // #define ONE_CIRCLE_PLUSES  1650(v=0.90) -> 660(v>=0.93)
  // #define TRACK_WIDTH  0.172
  // #define SET_MOTOR_DIR false

  // mainType:03 UGV Beast
  // #define WHEEL_D  0.0523
  // #define ONE_CIRCLE_PLUSES  1092
  // #define TRACK_WIDTH  0.141
  // #define SET_MOTOR_DIR true

  if (mainType == 1) {
    WHEEL_D = 0.0800;
    ONE_CIRCLE_PLUSES = 2100;
    TRACK_WIDTH = 0.125;
    SET_MOTOR_DIR = false;
  } else if (mainType == 2) {
    WHEEL_D = 0.0800;
    ONE_CIRCLE_PLUSES = 660;
    TRACK_WIDTH = 0.172;
    SET_MOTOR_DIR = false;
  } else if (mainType == 3) {
    WHEEL_D = 0.0523;
    ONE_CIRCLE_PLUSES = 1092;
    TRACK_WIDTH = 0.141;
    SET_MOTOR_DIR = true;
  }
  plusesRate = static_cast<float>(M_PI) * WHEEL_D / ONE_CIRCLE_PLUSES;

  if (mainType == 1) {
    screenLine_1 = "Rasp";
  } else if (mainType == 2) {
    screenLine_1 = "Rover";
  } else if (mainType == 3) {
    screenLine_1 = "Beast";
  }

  if (moduleType == 0) {
    screenLine_1 += " Null";
  } else if (moduleType == 1) {
    screenLine_1 += " Arm";
  } else if (moduleType == 2) {
    screenLine_1 += " PT";
  }
}
