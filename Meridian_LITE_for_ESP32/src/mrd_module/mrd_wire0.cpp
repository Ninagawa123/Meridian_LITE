// mrd_wire0.cpp - I2C wire0 handling implementation

#include "mrd_wire0.h"
#include <MPU6050_6Axis_MotionApps20.h> // non-inline definitions: include only in this TU
#include <Wire.h>

//------------------------------------------------------------------------------------
//  MPU6050専用データ (このTUにのみ閉じ込める)
//------------------------------------------------------------------------------------

struct Mpu6050State {
  MPU6050    mpu6050;
  uint8_t    mpuIntStatus;
  uint8_t    devStatus;
  uint16_t   packetSize;
  uint8_t    fifoBuffer[64];
  Quaternion q;
  VectorFloat gravity;
  float      ypr[3];
  VectorInt16 aa;
  VectorInt16 gyro;
  VectorInt16 mag;
};
static Mpu6050State mpu_state;

//------------------------------------------------------------------------------------
//  初期設定
//------------------------------------------------------------------------------------

bool mrd_wire0_init_i2c(int a_i2c0_speed, int a_pinSDA, int a_pinSCL) {
  Serial.print("Initializing wire0 I2C... ");
  if (a_pinSDA == -1 && a_pinSCL == -1) {
    Wire.begin();
  } else {
    Wire.begin(a_pinSDA, a_pinSCL);
  }
  Wire.setClock(a_i2c0_speed);
  return true;
}

bool mrd_wire0_init_mpu6050_dmp() {
  mpu_state.mpu6050.initialize();
  mpu_state.devStatus = mpu_state.mpu6050.dmpInitialize();

  mpu_state.mpu6050.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
  mpu_state.mpu6050.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
  mpu_state.mpu6050.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);
  mpu_state.mpu6050.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
  mpu_state.mpu6050.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
  mpu_state.mpu6050.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);

  if (mpu_state.devStatus == 0) {
    mpu_state.mpu6050.CalibrateAccel(6);
    mpu_state.mpu6050.CalibrateGyro(6);
    mpu_state.mpu6050.setDMPEnabled(true);
    mpu_state.packetSize = mpu_state.mpu6050.dmpGetFIFOPacketSize();
    Serial.println("MPU6050 OK.");
    return true;
  }
  Serial.println("IMU/AHRS DMP Initialization FAILED!");
  return false;
}

bool mrd_wire0_init_bno055(AhrsValue &a_ahrs) {
  if (!a_ahrs.bno.begin()) {
    Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    return false;
  }
  Serial.println("BNO055 mounted.");
  delay(50);
  a_ahrs.bno.setExtCrystalUse(false);
  delay(10);
  return true;
}

bool mrd_wire0_setup(ImuAhrsType a_imuahrs_type, int a_i2c0_speed, AhrsValue &a_ahrs,
                     int a_pinSDA, int a_pinSCL) {
  if (a_imuahrs_type > 0) {
    mrd_wire0_init_i2c(a_i2c0_speed, a_pinSDA, a_pinSCL);
  }
  if (a_imuahrs_type == IMU_TYPE_MPU6050) {
    return mrd_wire0_init_mpu6050_dmp();
  } else if (a_imuahrs_type == IMU_TYPE_BNO055) {
    return mrd_wire0_init_bno055(a_ahrs);
  }
  Serial.println("No IMU/AHRS sensor mounted.");
  return false;
}

//------------------------------------------------------------------------------------
//  センサデータの取得処理
//------------------------------------------------------------------------------------

void mrd_wire0_Core0_bno055_r(void *args) {
  while (1) {
    imu::Vector<3> accelerometer = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    ahrs.read[0] = (float)accelerometer.x();
    ahrs.read[1] = (float)accelerometer.y();
    ahrs.read[2] = (float)accelerometer.z();

    imu::Vector<3> gyroscope = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    ahrs.read[3] = gyroscope.x();
    ahrs.read[4] = gyroscope.y();
    ahrs.read[5] = gyroscope.z();

    imu::Vector<3> magnetometer = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    ahrs.read[6] = magnetometer.x();
    ahrs.read[7] = magnetometer.y();
    ahrs.read[8] = magnetometer.z();

    imu::Vector<3> euler = ahrs.bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    ahrs.read[12] = euler.y();
    ahrs.read[13] = euler.z();
    ahrs.yaw_source = euler.x();
    float yaw_tmp = euler.x() - ahrs.yaw_origin;
    if (yaw_tmp >= 180) {
      yaw_tmp -= 360;
    } else if (yaw_tmp < -180) {
      yaw_tmp += 360;
    }
    ahrs.read[14] = yaw_tmp;

    delay(IMUAHRS_INTERVAL);
  }
}

bool mrd_wire0_read_ahrs_i2c(AhrsValue &a_ahrs) {
  if (MOUNT_IMUAHRS == IMU_TYPE_MPU6050) {
    if (!mpu_state.mpu6050.dmpGetCurrentFIFOPacket(mpu_state.fifoBuffer)) {
      return false;
    }
    mpu_state.mpu6050.dmpGetQuaternion(&mpu_state.q, mpu_state.fifoBuffer);
    mpu_state.mpu6050.dmpGetGravity(&mpu_state.gravity, &mpu_state.q);
    mpu_state.mpu6050.dmpGetYawPitchRoll(mpu_state.ypr, &mpu_state.q, &mpu_state.gravity);

    mpu_state.mpu6050.dmpGetAccel(&mpu_state.aa, mpu_state.fifoBuffer);
    a_ahrs.read[0] = (float)mpu_state.aa.x;
    a_ahrs.read[1] = (float)mpu_state.aa.y;
    a_ahrs.read[2] = (float)mpu_state.aa.z;

    mpu_state.mpu6050.dmpGetGyro(&mpu_state.gyro, mpu_state.fifoBuffer);
    a_ahrs.read[3] = (float)mpu_state.gyro.x;
    a_ahrs.read[4] = (float)mpu_state.gyro.y;
    a_ahrs.read[5] = (float)mpu_state.gyro.z;

    a_ahrs.read[6]  = (float)mpu_state.mag.x;
    a_ahrs.read[7]  = (float)mpu_state.mag.y;
    a_ahrs.read[8]  = (float)mpu_state.mag.z;
    a_ahrs.read[9]  = mpu_state.gravity.x;
    a_ahrs.read[10] = mpu_state.gravity.y;
    a_ahrs.read[11] = mpu_state.gravity.z;

    a_ahrs.read[12] = mpu_state.ypr[2] * 180 / M_PI;
    a_ahrs.read[13] = mpu_state.ypr[1] * 180 / M_PI;
    a_ahrs.read[14] = (mpu_state.ypr[0] * 180 / M_PI) - a_ahrs.yaw_origin;
    a_ahrs.read[15] = 0;

    if (flg.imuahrs_available) {
      memcpy(a_ahrs.result, a_ahrs.read, sizeof(float) * 16);
    }
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------------
//  meriput
//------------------------------------------------------------------------------------

bool meriput90_ahrs(Meridim90Union &a_meridim, float a_ahrs_result[], int a_type) {
  if (a_type == IMU_TYPE_BNO055) {
    flg.imuahrs_available = false;
    a_meridim.sval[2]  = mrd.float2HfShort(a_ahrs_result[0]);
    a_meridim.sval[3]  = mrd.float2HfShort(a_ahrs_result[1]);
    a_meridim.sval[4]  = mrd.float2HfShort(a_ahrs_result[2]);
    a_meridim.sval[5]  = mrd.float2HfShort(a_ahrs_result[3]);
    a_meridim.sval[6]  = mrd.float2HfShort(a_ahrs_result[4]);
    a_meridim.sval[7]  = mrd.float2HfShort(a_ahrs_result[5]);
    a_meridim.sval[8]  = mrd.float2HfShort(a_ahrs_result[6]);
    a_meridim.sval[9]  = mrd.float2HfShort(a_ahrs_result[7]);
    a_meridim.sval[10] = mrd.float2HfShort(a_ahrs_result[8]);
    a_meridim.sval[11] = mrd.float2HfShort(a_ahrs_result[15]);
    a_meridim.sval[12] = mrd.float2HfShort(a_ahrs_result[12]);
    a_meridim.sval[13] = mrd.float2HfShort(a_ahrs_result[13]);
    a_meridim.sval[14] = mrd.float2HfShort(a_ahrs_result[14]);
    flg.imuahrs_available = true;
    return true;
  }
  return false;
}
