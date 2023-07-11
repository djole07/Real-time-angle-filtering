#include "KalmanMPU6050.h"


#define TIMER_10ms 1
#define TIMER_20ms 2
#define TIMER_100ms 10
#define TIMER_500ms 50

#define MAIN_LOOP_REFRESH_TIME 30

#define TAU 0.95

#define PIN_FORWARD 9
#define PIN_BACKWARD 10

#define SERIAL_IMU_DEBUG 1          // ako je na 0 onda necemo imati serijsku komunikaciju

#if SERIAL_IMU_DEBUG
#define DEBUG_INIT() Serial.begin(115200)
#define DEBUG_FLUSH() Serial.flush()
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
#define DEBUG_FLUSH()
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_TS_PRINT(x)
#define DEBUG_TS_PRINTLN(x)
#endif // SERIAL_IMU_DEBUG

byte value = 98;
volatile boolean timerCompleted = false;
volatile long counter = 0;
long brojac = 0;

double T = 20E-3;

double Kp = 5;
double Kd = 0;

double e = 0;
double de = 0;
double u = 0;
double pwm = 0;

double angleKal = 0;
double angleRawAcc = 0;
double angleRawGyro = 0;
double angleFil = 0;
double angleCompl = 0;
double angleComplMy = 0;

double angleKalPrev = 0;
double angleRawAccPrev = 0;
double angleRawGyroPrev = 0;
double angleRawPrev = 0;
double angleFilPrev = 0;
double angleComplPrev = 0;

double dangleKal = 0;
double dangleRawAcc = 0;
double dangleRawGyro = 0;
double dangleFil = 0;
double dangleCompl = 0;


double Ts = T;
double Tf = 0.05;
double p = exp(-Ts / Tf);

//double p = 0.1;

unsigned long timer1s = millis();
unsigned long vreme = millis();
unsigned long timer_main = millis();

bool isKalmanOn = true;

long elapsedTime = 0;

void setup()
{
  pinMode(PIN_FORWARD, OUTPUT);
  pinMode(PIN_BACKWARD, OUTPUT);

  // На почетку, моторе поставимо да не раде
  analogWrite(PIN_FORWARD, 0);
  analogWrite(PIN_BACKWARD, 0);

  // Почетак серијске комуникације
  DEBUG_INIT();

  IMU::init();
  IMU::read(true);

  // Иницијализација тајмера
  noInterrupts();                       // disable all interrupts
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = value;                        // preload timer
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 1024 prescaler
  TIMSK2 |= (1 << TOIE2);               // enable timer overflow interrupt ISR
  interrupts();                         // enable all interrupts
}

void loop()
{
  //IMU::read(isKalmanOn);

  //  if (millis() - timer1s >= 5000) {
  //    timer1s = millis();
  //    isKalmanOn = !isKalmanOn;
  //  }

  if (timerCompleted)
  {
    IMU::read(isKalmanOn);

    elapsedTime = micros();

    brojac++;
    angleRawAcc = IMU::getRawRollAcc();       // calculated only from accelerometer
    angleRawGyro = IMU::getRawRollGyro();     // calculated only from gyroscope
    angleCompl = IMU::getRawRollComplement();
    angleKal = IMU::getRoll();

    angleFil = p * angleFilPrev + (1 - p) * angleRawAcc;

    dangleRawAcc = (angleRawAcc - angleRawAccPrev) / T;
    dangleRawGyro = (angleRawGyro - angleRawGyroPrev) / T;
    dangleKal = (angleKal - angleKalPrev) / T;
    dangleFil = (angleFil - angleFilPrev) / T;
    dangleCompl = (angleCompl - angleComplPrev) / T;


    angleRawAccPrev = angleRawAcc;
    angleRawGyroPrev = angleRawGyro;
    angleComplPrev = angleCompl;
    angleKalPrev = angleKal;
    angleFilPrev = angleFil;
    timerCompleted = false;

    if (brojac < 600) {
      analogWrite(PIN_FORWARD, 40);
      analogWrite(PIN_BACKWARD, 0);
    }
    else {
      analogWrite(PIN_FORWARD, 0);
      analogWrite(PIN_BACKWARD, 0);
    }

    elapsedTime = micros() - elapsedTime;
    //    DEBUG_PRINT(" ");
    //    DEBUG_PRINT(elapsedTime);


    //DEBUG_PRINT(elapsedTime);
    //Serial.write(13);   // CR flag
    //Serial.write(10);   // LF flag
  }

  vreme = millis() - timer_main;
  // da malo sporije stampamo na monitor
  if (vreme >= MAIN_LOOP_REFRESH_TIME)
  {
    // Za arduino ploter
    DEBUG_PRINT("Akcelerometar:");
    DEBUG_PRINT(angleRawAcc);
    DEBUG_PRINT(" ");
    DEBUG_PRINT("Ziroskop:");
    DEBUG_PRINT(angleRawGyro);
    DEBUG_PRINT(" ");
    DEBUG_PRINT("Komplemetarni:");
    DEBUG_PRINT(angleCompl);
    DEBUG_PRINT(" ");
    DEBUG_PRINT("Kalman:");
    DEBUG_PRINT(angleKal);
    DEBUG_PRINT(" ");
    DEBUG_PRINT("Prvi_red:");
    DEBUG_PRINT(angleFil);
    DEBUG_PRINT(" ");

    DEBUG_PRINT("dAkcelerometar:");
    DEBUG_PRINT(dangleRawAcc);
    DEBUG_PRINT(" ");
    DEBUG_PRINT("dZiroskop:");
    DEBUG_PRINT(dangleRawGyro);
    DEBUG_PRINT(" ");
    DEBUG_PRINT("dKomplemetarni:");
    DEBUG_PRINT(dangleCompl);
    DEBUG_PRINT(" ");
    DEBUG_PRINT("dKalman:");
    DEBUG_PRINT(dangleKal);
    DEBUG_PRINT(" ");
    DEBUG_PRINT("dPrvi_red:");
    DEBUG_PRINT(dangleFil);
    DEBUG_PRINT(" ");

    Serial.write(13);   // CR flag
    Serial.write(10);   // LF flag

    timer_main = millis();
  }

}



ISR(TIMER2_OVF_vect)                    // interrupt service routine for overflow
{
  if (++counter == TIMER_20ms) {
    timerCompleted = true;
    counter = 0;
  }

  TCNT2 = value;// preload timer
}
