#include "IMU.h"
#include "config.h"
#include "globals.h"
#include <Wire.h>
#include <sensor_msgs/msg/imu.h>

// --- CORRECTION : Instanciation des variables globales ---
EulerAngles stAngles;
IMU_ST_SENSOR_DATA_FLOAT stGyroRawData;
IMU_ST_SENSOR_DATA_FLOAT stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;
// --------------------------------------------------------

void calibrateMagn();
void imuAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,
                   float mx, float my, float mz);
float invSqrt(float x);

AK09918_err_type_t err;
QMI8658 qmi8658_;
AK09918 magnetometer_;

int16_t offset_x = -12, offset_y = 0, offset_z = 0;
int16_t x, y, z;
double declination_shenzhen = -3.22;

#define Kp 4.50f
#define Ki 1.0f

float q0, q1, q2, q3;

// Message IMU partagé (défini dans lidar_manager.cpp, publié par
// microRosLidarTask)
extern sensor_msgs__msg__Imu imu_msg;

void imuInit() {
  // Wire.begin() est appelé dans imuTask() (même core)

  if (qmi8658_.begin() == 0)
    Serial.println("qmi8658_init fail");

  if (magnetometer_.initialize())
    Serial.println("AK09918_init fail");

  magnetometer_.switchMode(AK09918_CONTINUOUS_100HZ);
  err = magnetometer_.isDataReady();

  int retry_times = 0;
  while (err != AK09918_ERR_OK) {
    delay(100);
    magnetometer_.reset();
    delay(100);
    magnetometer_.switchMode(AK09918_CONTINUOUS_100HZ);
    err = magnetometer_.isDataReady();
    retry_times++;
    if (retry_times > 10)
      break;
  }

  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
}

void imuDataGet(EulerAngles *pstAngles,
                IMU_ST_SENSOR_DATA_FLOAT *pstGyroRawData,
                IMU_ST_SENSOR_DATA_FLOAT *pstAccelRawData,
                IMU_ST_SENSOR_DATA *pstMagnRawData) {
  float acc[3], gyro[3];
  float MotionVal[9];

  magnetometer_.getData(&x, &y, &z);

  pstMagnRawData->s16X = x - offset_x;
  pstMagnRawData->s16Y = y - offset_y;
  pstMagnRawData->s16Z = z - offset_z;

  qmi8658_.read_sensor_data(acc, gyro);

  MotionVal[0] = gyro[0];
  MotionVal[1] = gyro[1];
  MotionVal[2] = gyro[2];
  MotionVal[3] = acc[0];
  MotionVal[4] = acc[1];
  MotionVal[5] = acc[2];
  MotionVal[6] = pstMagnRawData->s16X;
  MotionVal[7] = pstMagnRawData->s16Y;
  MotionVal[8] = pstMagnRawData->s16Z;

  imuAHRSupdate((float)MotionVal[0] * 0.0175, (float)MotionVal[1] * 0.0175,
                (float)MotionVal[2] * 0.0175, (float)MotionVal[3],
                (float)MotionVal[4], (float)MotionVal[5], (float)MotionVal[6],
                (float)MotionVal[7], MotionVal[8]);

  pstAngles->pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
  pstAngles->roll =
      atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
  pstAngles->yaw =
      atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.3;

  pstGyroRawData->X = gyro[0];
  pstGyroRawData->Y = gyro[1];
  pstGyroRawData->Z = gyro[2];

  pstAccelRawData->X = acc[0];
  pstAccelRawData->Y = acc[1];
  pstAccelRawData->Z = acc[2];
}

// Fonction de mise à jour pour le thread externe
void updateIMUData() {
  // Plus d'erreur ici car stAngles est maintenant du type EulerAngles
  imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
}

void imuAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,
                   float mx, float my, float mz) {
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;
  float ex, ey, ez, halfT = 0.024f;

  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;

  norm = invSqrt(ax * ax + ay * ay + az * az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;

  norm = invSqrt(mx * mx + my * my + mz * mz);
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) +
       2 * mz * (q1q3 + q0q2);
  hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) +
       2 * mz * (q2q3 - q0q1);
  hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) +
       2 * mz * (0.5f - q1q1 - q2q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = hz;

  vx = 2 * (q1q3 - q0q2);
  vy = 2 * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
  wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
  wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2);

  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

  if (ex != 0.0f && ey != 0.0f && ez != 0.0f) {
    exInt = exInt + ex * Ki * halfT;
    eyInt = eyInt + ey * Ki * halfT;
    ezInt = ezInt + ez * Ki * halfT;

    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;
  }

  q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
  q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
  q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
  q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

  norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
}

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void calibrateMagn(void) {
  int16_t temp[9];
  Serial.printf("keep 10dof-imu device horizontal\n");
  delay(2000);
  magnetometer_.getData(&x, &y, &z);
  temp[0] = x;
  temp[1] = y;
  temp[2] = z;

  Serial.printf("rotate z axis 180 degrees\n");
  delay(2000);
  magnetometer_.getData(&x, &y, &z);
  temp[3] = x;
  temp[4] = y;
  temp[5] = z;

  Serial.printf("flip device\n");
  delay(2000);
  magnetometer_.getData(&x, &y, &z);
  temp[6] = x;
  temp[7] = y;
  temp[8] = z;

  offset_x = (temp[0] + temp[3]) / 2;
  offset_y = (temp[1] + temp[4]) / 2;
  offset_z = (temp[5] + temp[8]) / 2;
}

void imuTask(void *pvParameters) {

  Serial.println("[IMU] Task started, init I2C on this core...");

  // Init I2C sur le même core que la tâche
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Serial.println("[IMU] Wire OK");

  // Init capteurs IMU
  imuInit();
  Serial.println("[IMU] IMU sensors initialized");

  // Zero-init du message IMU
  memset(&imu_msg, 0, sizeof(imu_msg));

  imu_msg.header.frame_id.data = (char *)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  // Covariances inconnues (convention ROS: -1 en [0])
  imu_msg.orientation_covariance[0] = -1.0;
  imu_msg.angular_velocity_covariance[0] = -1.0;
  imu_msg.linear_acceleration_covariance[0] = -1.0;

  Serial.println("[IMU] Init complete, entering sensor loop");

  while (true) {
    updateIMUData();

    // Ajout  Pure Pursuit : Récupération de l'orientation du robot
    robotPose.theta = stAngles.yaw;

    // Remplir le message partagé (publié par microRosLidarTask)
    imu_msg.orientation.x = q1;
    imu_msg.orientation.y = q2;
    imu_msg.orientation.z = q3;
    imu_msg.orientation.w = q0;

    imu_msg.angular_velocity.x = stGyroRawData.X;
    imu_msg.angular_velocity.y = stGyroRawData.Y;
    imu_msg.angular_velocity.z = stGyroRawData.Z;

    imu_msg.linear_acceleration.x = stAccelRawData.X;
    imu_msg.linear_acceleration.y = stAccelRawData.Y;
    imu_msg.linear_acceleration.z = stAccelRawData.Z;

    // PAS de rcl_publish ici — c'est microRosLidarTask qui publie
    vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
  }
}
