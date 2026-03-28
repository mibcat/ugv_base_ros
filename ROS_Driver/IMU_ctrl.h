#include <ICM20948_WE.h>

ICM20948_WE myICM = ICM20948_WE();

void imu_init() {
  if (myICM.init()) {
    // execute auto calibration
    // after auto calibration range and dlpf are as follows:
    // acc: range = ±2g, dlpf = 6
    // gyro: range = ±250dps, dlpf = 6
    myICM.autoOffsets();

    // currently only a simple implementation
    myICM.enableFifo(false);
    myICM.enableLowPower(false);

    // set acc and gyro range and dlpf according to the application
    // myICM.setAccRange(ICM20948_ACC_RANGE_2G);
    // myICM.setAccDLPF(ICM20948_DLPF_6);
    // myICM.setGyrRange(ICM20948_GYRO_RANGE_250);
    // myICM.setGyrDLPF(ICM20948_DLPF_6);

    // setup magnetometer
    if (myICM.initMagnetometer()) {
      myICM.setMagOpMode(AK09916_CONT_MODE_10HZ);
      Serial.println(F("Magnetometer initialized successfully."));
    } else {
      Serial.println(F("Magnetometer initialization failed."));
    }
    Serial.println(F("IMU initialized successfully."));
  } else {
    Serial.println(F("IMU initialization failed."));
  }
}

void updateIMUData() {
  xyzFloat value;
  // get raw acc and gyro data
  myICM.readSensor();

  // get corrected acc g values in [g]
  myICM.getGValues(&value);
  ax = value.x;
  ay = value.y;
  az = value.z;

  // get corrected gyro values in [dps]
  myICM.getGyrValues(&value);
  gx = value.x;
  gy = value.y;
  gz = value.z;

  // get magnetometer values in [uT]
  myICM.getMagValues(&value);
  mx = value.x;
  my = value.y;
  mz = value.z;
}

// {"T":127}
// reset qc0 ~ q3
void imuCalibration() {
  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;

  // execute auto calibration
  myICM.autoOffsets();

  xyzFloat gyrBias = myICM.getGyrOffsets();
  jsonInfoHttp["gx"] = gyrBias.x;
  jsonInfoHttp["gy"] = gyrBias.y;
  jsonInfoHttp["gz"] = gyrBias.z;

  xyzFloat accBias = myICM.getAccOffsets();
  jsonInfoHttp["ax"] = accBias.x;
  jsonInfoHttp["ay"] = accBias.y;
  jsonInfoHttp["az"] = accBias.z;

  jsonInfoHttp["cx"] = 0.0;
  jsonInfoHttp["cy"] = 0.0;
  jsonInfoHttp["cz"] = 0.0;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);

  qc0 = 1.0;
  qc1 = 0.0;
  qc2 = 0.0;
  qc3 = 0.0;
}

// {"T":126}
void getIMUData() {
  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = FEEDBACK_IMU_DATA;

  jsonInfoHttp["r"] = icm_roll;
  jsonInfoHttp["p"] = icm_pitch;
  jsonInfoHttp["y"] = icm_yaw;

  jsonInfoHttp["q0"] = q0;
  jsonInfoHttp["q1"] = q1;
  jsonInfoHttp["q2"] = q2;
  jsonInfoHttp["q3"] = q3;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}

// {"T":128}
// get and set qc0 ~ qc3 compensation quaternion
void getIMUOffset() {
  double halfRoll = -icm_roll / 2.0;
  double qr0 = cos(halfRoll);
  double qr1 = sin(halfRoll);
  double qr2 = 0.0;
  double qr3 = 0.0;

  double halfPitch = -icm_pitch / 2.0;
  double qp0 = cos(halfPitch);
  double qp1 = 0.0;
  double qp2 = sin(halfPitch);
  double qp3 = 0.0;

  qc0 = qr0 * qp0 - qr1 * qp1 - qr2 * qp2 - qr3 * qp3;
  qc1 = qr0 * qp1 + qr1 * qp0 + qr2 * qp3 - qr3 * qp2;
  qc2 = qr0 * qp2 - qr1 * qp3 + qr2 * qp0 + qr3 * qp1;
  qc3 = qr0 * qp3 + qr1 * qp2 - qr2 * qp1 + qr3 * qp0;

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;
  jsonInfoHttp["qc0"] = qc0;
  jsonInfoHttp["qc1"] = qc1;
  jsonInfoHttp["qc2"] = qc2;
  jsonInfoHttp["qc3"] = qc3;
  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}

// {"T":129,"gx":0,"gy":0,"gz":0,"ax":0,"ay":0,"az":0,"cx":0,"cy":0,"cz":0}
void setIMUOffset(int32_t inGX, int32_t inGY, int32_t inGZ, int32_t inAX,
                  int32_t inAY, int32_t inAZ, int32_t inCX, int32_t inCY,
                  int32_t inCZ) {
  // set gyro offset
  xyzFloat gyrBias(inGX, inGY, inGZ);
  myICM.setGyrOffsets(gyrBias);

  // set acc offset
  xyzFloat accBias(inAX, inAY, inAZ);
  myICM.setAccOffsets(accBias);

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;

  jsonInfoHttp["status"] = 1;

  jsonInfoHttp["gx"] = gyrBias.x;
  jsonInfoHttp["gy"] = gyrBias.y;
  jsonInfoHttp["gz"] = gyrBias.z;

  jsonInfoHttp["ax"] = accBias.x;
  jsonInfoHttp["ay"] = accBias.y;
  jsonInfoHttp["az"] = accBias.z;

  jsonInfoHttp["cx"] = 0.0;
  jsonInfoHttp["cy"] = 0.0;
  jsonInfoHttp["cz"] = 0.0;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}