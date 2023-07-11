/*
  Sa ziroskopa citamo sa 250deg/s
  Sa akcelerometra citamo sa +-2g
*/

#include "KalmanMPU6050.h"

#include <Wire.h>

#if SERIAL_IMU_DEBUG
#define DEBUG_INIT() Serial.begin(115200)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_TS_PRINT(x)  \
  DEBUG_PRINT_TIMESTAMP(); \
  Serial.print(x)
#define DEBUG_TS_PRINTLN(x) \
  DEBUG_PRINT_TIMESTAMP();  \
  Serial.println(x)
#else
#define DEBUG_INIT()
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_TS_PRINT(x)
#define DEBUG_TS_PRINTLN(x)
#endif // SERIAL_IMU_DEBUG

#ifndef M_PI
#define M_PI 3.14159265359
#endif // M_PI
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 180.0 / M_PI
#endif // RAD_TO_DEG

// 250 deg/s --> 131.0, 500 deg/s --> 65.5, 1000 deg/s --> 32.8, 2000 deg/s --> 16.4
#define SCALE_FACTOR_GYRO 131.0 // faktor skaliranja za ziroskop. Mi smo podesili da je preciznost 250deg/s
// 2g --> 16384 , 4g --> 8192 , 8g --> 4096, 16g --> 2048
#define SCALE_FACTOR_ACC 16384  /* faktor skaliranja za akcelerometar. Zapravo nam nece biti potreban, 
  posto radimo arctg od 2 broja koja bismo trebali da skaliramo. A ako oba preskaliramo, onda ce se taj faktor skaliranja skratiti
  pa ne mroamo to da radimo
*/

#define TAU 0.67
#define sqr(x) x *x
#define hypotenuse(x, y) sqrt(sqr(x) + sqr(y))

// MPU-6050
#define IMU_ADDR 0x68
#define IMU_ACCEL_XOUT_H 0x3B
#define IMU_REG 0x19
#define IMU_PWR_MGMT_1 0x6B

typedef struct kalman_t
{
  double Q_angle;   // Varijansa suma sa akcelerometra
  double Q_bias;    // Varijansa suma pomeranja nule ziroskopa
  double R_measure; // Merena varijansa suma - ovo je zapravo varijansa suma izmerenih podataka
  double angle; // Ugao izracunat preko Kalmanavog filtra - deo 2x1 matrice
  double bias;  // Bias ziroskopa izracunat preko Kalmanovog filtra - deo 2x1 matrice
  double rate;  // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

  double P[2][2]; // Matrica greske kovarijacije
  double K[2];    // Kalmanovo pojacanje
  double y;       // Ugaona razlika
  double S;       // Procenjena greska
} Kalman;

// Kalman Variables

static Kalman kalmanX; // Create the Kalman instances
static Kalman kalmanY;

static double gyroXAngle, gyroYAngle; // ugao racunat samo pomocu ziroskopa

uint32_t IMU::lastProcessed = 0;

int16_t IMU::accelX, IMU::accelY, IMU::accelZ;
int16_t IMU::gyroX,  IMU::gyroY,  IMU::gyroZ;

double IMU::kalXAngle, IMU::kalYAngle;

// Complementary filter
double IMU::rollComplement, IMU::rollRaw;
double IMU::gyroRoll;
double IMU::gyroRollRaw;
double IMU::gyroYNorm;

// Kalman Function Definition

inline void Kalman_Init(Kalman *kalPointer)
{
  /* Postavljamo inicijalne vrednosti za matrice Q i R koje korisnik moze kasnije da menja */
  kalPointer->Q_angle = 0.001;
  kalPointer->Q_bias = 0.003;
  kalPointer->R_measure = 0.03;

  kalPointer->angle = 0; // Resetujemo vrednost za ugao
  kalPointer->bias = 0;  // Resetujemo vrednost za bias

  kalPointer->P[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
  kalPointer->P[0][1] = 0;
  kalPointer->P[1][0] = 0;
  kalPointer->P[1][1] = 0;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
inline double Kalman_GetAngle(Kalman *kalPointer,
                              double newAngle, double newRate, double dt)
{
  // Discrete Kalman filter time update equations - Time Update ("Predict")
  // Update xhat - Project the state ahead
  /* Step 1 */
  kalPointer->rate = newRate - kalPointer->bias;
  kalPointer->angle += dt * kalPointer->rate;

  // Update estimation error covariance - Project the error covariance ahead
  /* Step 2 */
  kalPointer->P[0][0] += dt * (dt * kalPointer->P[1][1] - kalPointer->P[0][1] -
                               kalPointer->P[1][0] + kalPointer->Q_angle);
  kalPointer->P[0][1] -= dt * kalPointer->P[1][1];
  kalPointer->P[1][0] -= dt * kalPointer->P[1][1];
  kalPointer->P[1][1] += kalPointer->Q_bias * dt;

  // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
  // Calculate Kalman gain - Compute the Kalman gain
  /* Step 4 */
  kalPointer->S = kalPointer->P[0][0] + kalPointer->R_measure;
  /* Step 5 */
  kalPointer->K[0] = kalPointer->P[0][0] / kalPointer->S;
  kalPointer->K[1] = kalPointer->P[1][0] / kalPointer->S;

  // Calculate angle and bias - Update estimate with measurement zk (newAngle)
  /* Step 3 */
  kalPointer->y = newAngle - kalPointer->angle;
  /* Step 6 */
  kalPointer->angle += kalPointer->K[0] * kalPointer->y;
  kalPointer->bias += kalPointer->K[1] * kalPointer->y;

  // Calculate estimation error covariance - Update the error covariance
  /* Step 7 */
  kalPointer->P[0][0] -= kalPointer->K[0] * kalPointer->P[0][0];
  kalPointer->P[0][1] -= kalPointer->K[0] * kalPointer->P[0][1];
  kalPointer->P[1][0] -= kalPointer->K[1] * kalPointer->P[0][0];
  kalPointer->P[1][1] -= kalPointer->K[1] * kalPointer->P[0][1];

  return kalPointer->angle;
};

void IMU::init()
{
  DEBUG_INIT();
  Wire.begin();
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(IMU_REG);
  Wire.write(7);
  for (byte i = 0; i < 3; i++)
  {
    Wire.write(0);
  }
  Wire.endTransmission(false);

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(IMU_PWR_MGMT_1);
  Wire.write(0x01);
  Wire.endTransmission(true);

  delay(100);

  Kalman_Init(&kalmanX);
  Kalman_Init(&kalmanY);

  MPU6050Read();

  double roll, pitch;
  IMU::RollPitchFromAccel(&roll, &pitch);

  kalmanX.angle = roll; // Set starting angle
  kalmanY.angle = pitch;
  gyroXAngle = roll;
  gyroYAngle = pitch;

  lastProcessed = micros();
  DEBUG_TS_PRINTLN("Finished IMU setup.");
}

void IMU::read(bool isKalmanOn)
{
  static double dt = 0;

  MPU6050Read();

  dt = (double)(micros() - lastProcessed) / 1000000;
  lastProcessed = micros();

  double roll, pitch;
  IMU::RollPitchFromAccel(&roll, &pitch);
  rollRaw = roll;   // for complementary filter

  double gyroXRate, gyroYRate;
  gyroXRate = (double)gyroX / 131.0; // Convert to deg/s
  gyroYRate = (double)gyroY / 131.0; // Convert to deg/s

  gyroYNorm = gyroYRate;
  
  gyroRoll = gyroRoll - gyroYRate * dt; // ugao izracunat samo preko ziroskopa
  gyroRollRaw = gyroRollRaw - (gyroYRate) * dt;
  //rollComplement = TAU * (rollComplement - gyroYRate * dt) + (1 - TAU) * (roll);
  gyroRoll = TAU * gyroRoll + (1 - TAU) * (roll); // korekcija ziroskopa
  //rollComplement = TAU * gyroRoll + (1 - TAU) * (roll); // korekcija ziroskopa

  rollComplement = gyroRoll;

  if (isKalmanOn) {

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalXAngle > 90) ||
        (roll > 90 && kalXAngle < -90))
    {
      kalmanX.angle = roll;
      kalXAngle = roll;
      gyroXAngle = roll;
    }
    else
    {
      kalXAngle = Kalman_GetAngle(&kalmanX, roll, gyroXRate, dt); // Calculate the angle using a Kalman filter
    }

    if (abs(kalXAngle) > 90)
      gyroYRate = -gyroYRate; // Invert rate, so it fits the restriced accelerometer reading
    kalYAngle = Kalman_GetAngle(&kalmanY, pitch, gyroYRate, dt);
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalYAngle > 90) ||
        (pitch > 90 && kalYAngle < -90))
    {
      kalmanY.angle = pitch;
      kalYAngle = pitch;
      gyroYAngle = pitch;
    }
    else
    {
      kalYAngle = Kalman_GetAngle(&kalmanY, pitch, gyroYRate, dt); // Calculate the angle using a Kalman filter
    }

    if (abs(kalYAngle) > 90)
      gyroXRate = -gyroXRate;                                   // Invert rate, so it fits the restriced accelerometer reading
    kalXAngle = Kalman_GetAngle(&kalmanX, roll, gyroXRate, dt); // Calculate the angle using a Kalman filter
#endif

    gyroXAngle += gyroXRate * dt; // Calculate gyro angle without any filter
    gyroYAngle += gyroYRate * dt;
    //gyroXAngle += kalmanX.rate * dt; // Calculate gyro angle using the unbiased rate
    //gyroYAngle += kalmanY.rate * dt;

    // Reset the gyro angle when it has drifted too much
    if (gyroXAngle < -180 || gyroXAngle > 180)
      gyroXAngle = kalXAngle;
    if (gyroYAngle < -180 || gyroYAngle > 180)
      gyroYAngle = kalYAngle;

    //rollComplement = TAU * (rollComplement - gyroYRate * dt) + (1 - TAU) * (roll);

    DEBUG_TS_PRINT("KalAngleX: ");
    DEBUG_PRINTLN(kalXAngle);
    DEBUG_TS_PRINT("KalAngleY: ");
    DEBUG_PRINTLN(kalYAngle);
  }// kraj if-a za isKalmanOn
}

uint32_t IMU::getLastReadTime()
{
  return lastProcessed;
}

int16_t IMU::getRawAccelX()
{
  return accelX;
}

int16_t IMU::getRawAccelY()
{
  return accelY;
}

int16_t IMU::getRawAccelZ()
{
  return accelZ;
}

int16_t IMU::getRawGyroX()
{
  return gyroX;
}

int16_t IMU::getRawGyroY()
{
  return gyroY;
}

int16_t IMU::getRawGyroZ()
{
  return gyroZ;
}

double IMU::getRoll()
{
  return kalXAngle;
}

double IMU::getPitch()
{
  return kalYAngle;
}

// IMU Function Definition

void IMU::MPU6050Read()
{
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(IMU_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, 14, true);

  accelX = (int16_t)(Wire.read() << 8 | Wire.read());
  accelY = (int16_t)(Wire.read() << 8 | Wire.read());
  accelZ = (int16_t)(Wire.read() << 8 | Wire.read());
  Wire.read();
  Wire.read(); // Temperature
  gyroX = (int16_t)(Wire.read() << 8 | Wire.read());
  gyroY = (int16_t)(Wire.read() << 8 | Wire.read());
  gyroZ = (int16_t)(Wire.read() << 8 | Wire.read());

  DEBUG_TS_PRINT("Raw AccelX: ");
  DEBUG_PRINTLN(accelX);
  DEBUG_TS_PRINT("Raw AccelY: ");
  DEBUG_PRINTLN(accelY);
  DEBUG_TS_PRINT("Raw AccelZ: ");
  DEBUG_PRINTLN(accelZ);
  DEBUG_TS_PRINT("Raw GyroX: ");
  DEBUG_PRINTLN(gyroX);
  DEBUG_TS_PRINT("Raw GyroY: ");
  DEBUG_PRINTLN(gyroY);
  DEBUG_TS_PRINT("Raw GyroZ: ");
  DEBUG_PRINTLN(gyroZ);
}

void IMU::RollPitchFromAccel(double *roll, double *pitch)
{
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  *roll = atan2((double)accelY, (double)accelZ) * RAD_TO_DEG;
  *pitch = atan((double) - accelX / hypotenuse((double)accelY, (double)accelZ)) * RAD_TO_DEG;
#else  // Eq. 28 and 29
  *roll = atan((double)accelY / hypotenuse((double)accelX, (double)accelZ)) * RAD_TO_DEG;
  *pitch = atan2((double) - accelX, (double)accelZ) * RAD_TO_DEG;
#endif // RESTRICT_PITCH

  DEBUG_TS_PRINT("Accelerometer Measured Roll: ");
  DEBUG_PRINTLN(roll);
  DEBUG_TS_PRINT("Accelerometer Measured Pitch: ");
  DEBUG_PRINTLN(pitch);
}
double IMU::getRawRollAcc()
{ // calculating roll only from accelerometer
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  return atan2((double)accelY, (double)accelZ) * RAD_TO_DEG;
#else  // Eq. 28 and 29
  return atan((double)accelY / hypotenuse((double)accelX, (double)accelZ)) * RAD_TO_DEG;
#endif // RESTRICT_PITCH
}

double IMU::getRawRollGyro()
{ // calculating roll only from gyro
  return gyroRollRaw;  // it's necessary to call IMU::read() before calling this function
}

// prva verzija za testiranje
//double IMU::getRawRollComplement()
//{
//  double accelRoll = atan2((double)accelY, (double)accelZ) * RAD_TO_DEG;
//  return tau * (roll - gyroY/SCALE_FACTOR_GYRO * dtPublic) + (1 - tau)*(accelRoll);
//}
double IMU::getRawRollComplement()
{
  return rollComplement;
}
