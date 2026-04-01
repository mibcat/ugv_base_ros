#include <math.h>

// ============ Adaptive Measurement Window Parameters ============

// 100ms - minimum window for fast response at high speeds
const unsigned long WINDOW_US_MIN = 100000;
// 250ms - maximum window for low speed turns, optimized for in-place rotations
const unsigned long WINDOW_US_MAX = 250000;
// 0.3 m/s - speed threshold for using minimum window
const float SPEED_FOR_MIN_WINDOW = 0.30;
// 0.15 m/s - speed threshold for using maximum window
const float SPEED_FOR_MAX_WINDOW = 0.15;

// Current adaptive measurement window in microseconds
static unsigned long adaptiveWindowUs = 100000;

bool usePIDCompute = true;
float speedFactorA = 1.0;
float speedFactorB = 1.0;
bool heartbeatStopFlag = false;
bool emergencyStopActive = false;
static unsigned long lastWheelSpeedMeasureTime{};

/**
 * @brief Update motion control flags.
 *
 * This function updates the motion control flags based on the current control
 * mode. It returns false if the emergency stop is active, preventing motion
 * commands from being processed. Otherwise, it returns true, allowing motion
 * command processing to proceed.
 *
 * @param pidMode True if PID control should be used (ROS control mode), false
 * for direct PWM control.
 * @return True if motion command processing can proceed, false if emergency
 * stop is active.
 */
bool updateMotionControlFlags(bool pidMode) {
  if (emergencyStopActive) {
    // motion commands ignored during emergency stop
    return false;
  }

  // enable or disable PID compute based on control mode (ROS control vs. direct
  // PWM)
  usePIDCompute = pidMode;
  // reset heartbeat timer
  heartbeatStopFlag = false;
  lastCmdRecvTime = millis();
  // motion command processing can proceed
  return true;
}

void switchEmergencyStop() {
  // Cut motor power immediately
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

/** current left speeds [m/s] */
float speedGetA;
/** current right speeds [m/s] */
float speedGetB;

/** distance per pulse [m/pulse] */
float plusesRate = static_cast<float>(M_PI) * WHEEL_D / ONE_CIRCLE_PLUSES;

void initEncoders() {
  encoderA.attachHalfQuad(AENCA, AENCB);
  encoderB.attachHalfQuad(BENCA, BENCB);
  encoderA.setCount(0);
  encoderB.setCount(0);
}

/**
 * @brief Calculate adaptive measurement window.
 *
 * This function calculates the adaptive measurement window based on the current
 * wheel speeds. It uses linear interpolation between SPEED_FOR_MAX_WINDOW and
 * SPEED_FOR_MIN_WINDOW depending on the the current maximum absolute wheel
 * speed (speedGetA and speedGetB). The resulting adaptiveWindowUs is set to a
 * value between WINDOW_US_MAX and WINDOW_US_MIN, allowing for faster feedback
 * at high speeds and better accuracy at low speeds.
 */
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

/**
 * @brief Get wheel speeds.
 *
 * This function calculates the current wheel speeds based on encoder counts
 * and updates the adaptive measurement window accordingly.
 */
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

// Important note to implement a full reset of the PID controller:
// 1) reset the global variables outputA/B and setpointA/B to zero
// 2) the internal PID states are only reset when mode is changing from manual
//    to automatic, so we need to set the mode to manual before calling Start()
//    which is changing the mode back to automatic.

void pidControllerInit() {
  outputA = 0.0;
  setpointA = 0.0;
  pidA.SetMode(PID::Manual);
  pidA.Start(speedGetA, outputA, setpointA);
  pidA.SetOutputLimits(-255, 255);
  pidA.SetMode(PID::Automatic);

  outputB = 0.0;
  setpointB = 0.0;
  pidB.SetMode(PID::Manual);
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
  // Cannot set speed while emergency stop is active
  if (emergencyStopActive) {
    return;
  }

  if (inputLeft < -2.0 || inputLeft > 2.0) {
    return;
  }

  if (inputRight < -2.0 || inputRight > 2.0) {
    return;
  }

  // Calculate scaled target setpoints
  setpointA = inputLeft * speedFactorA;
  setpointB = inputRight * speedFactorB;

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
  float goalA = rosX - (rosZ * effective_track_width / 2.0);
  float goalB = rosX + (rosZ * effective_track_width / 2.0);

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

  // [m/inc] depends on wheel diameter and encoder resolution
  plusesRate = static_cast<float>(M_PI) * WHEEL_D / ONE_CIRCLE_PLUSES;
  // although will not work well without having a correction factor
  effective_track_width = TRACK_WIDTH;

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

/**
 * @brief Activate emergency stop.
 *
 * This function sets the emergency stop flag and immediately cuts power to the
 * motors.
 */
void setMotionEmergencyStop() {
  emergencyStopActive = true;
  usePIDCompute = false;
  switchEmergencyStop();
}

// Due to bad SW design, the location of this function must be after all the
// declarations of the global variables it uses, so it is placed at the end of
// this file together with the setMotionEmergencyStop() function.

/**
 * @brief Reset emergency stop.
 *
 * This function clears the emergency stop flag, reinitializes the PID
 * controllers, and resets the ramped setpoints to zero for a smooth restart.
 */
void resetMotionEmergencyStop() {
  // reinitialize PID controllers to reset internal state
  pidControllerInit();

  // clear emergency stop flag to allow motion commands to be processed again
  emergencyStopActive = false;
}
