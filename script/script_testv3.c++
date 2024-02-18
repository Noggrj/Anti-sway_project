#include "TimerOne.h"

#define runEvery(t) for (static uint16_t _lasttime; (uint16_t)((uint16_t)millis() - _lasttime) >= (t); _lasttime += (t))
#define runEveryus(t) for (static uint16_t _lasttime; (uint16_t)((uint16_t)micros() - _lasttime) >= (t); _lasttime += (t))

#define PINO_A_ENCODER 2
#define PINO_B_ENCODER 3
#define PINO_PWM 9
#define PINO_PWM_2 10

const unsigned Ts = 5000;
const unsigned PWM_FREQ = 10;
const double VFONTE = 5.0;

volatile long _pulsosencoder[2];

double r = 0, s, S;
bool Q = false, flag, MA = false;

double w[2];
double theta_ant[2];
unsigned long tempo[2], deltat[2];

uint16_t a = 0x01;
unsigned short n = 1, m = 10;

volatile byte abOld[2];
volatile byte changedCount[2];
const unsigned NENC = 2;
const unsigned MASCARA = 0x0c;
const double ENC_RES = 32.0;
int u[2] = {0};

void setup() {
  Serial.begin(2400);
  pinMode(PINO_PWM, OUTPUT);
  pinMode(PINO_PWM_2, OUTPUT);
  pinMode(PINO_A_ENCODER, INPUT_PULLUP);
  pinMode(PINO_B_ENCODER, INPUT_PULLUP);
  digitalWrite(PINO_PWM, LOW);
  digitalWrite(PINO_PWM_2, LOW);
  analogReference(INTERNAL);

  setaEncoders(INPUT_PULLUP, true);
  Timer1.initialize(1000 / PWM_FREQ);
  Timer1.pwm(PINO_PWM, 0);
  Timer1.pwm(PINO_PWM_2, 0);
}

void loop() {
  runEvery(50) { leSerial(); }

  runEveryus(Ts) {
    if (n < m)
      n++;
    else {
      a = geraPRBS11(a);
      n = 1;
    }
    u[0] = (a & 1) ? 8 : 4;
    u[1] = (a & 1) ? 8 : 4;

    runEvery(500) {
      if (u[0] < 11)
        u[0]++;
      else
        u[0] = 0;

      if (u[1] < 11)
        u[1]++;
      else
        u[1] = 0;
    }

    runEvery(500) {
      if (u[0] != 4)
        u[0] = 4;
      else
        u[0] = 8;

      if (u[1] != 4)
        u[1] = 4;
      else
        u[1] = 8;
    }

    w[0] = leang(0);
    w[1] = leang(1);
    acionaMotor(5, 0);
    acionaMotor(5, 1);
    enviaSerial();
  }
}

uint16_t geraPRBS7(uint16_t a) {
  uint16_t newbit = (((a >> 6) ^ (a >> 5)) & 1);
  return a = ((a << 1) | newbit) & 0x7f;
}

uint16_t geraPRBS9(uint16_t a) {
  uint16_t newbit = (((a >> 8) ^ (a >> 4)) & 1);
  return a = ((a << 1) | newbit) & 0x1ff;
}

uint16_t geraPRBS11(uint16_t a) {
  uint16_t newbit = (((a >> 10) ^ (a >> 8)) & 1);
  return a = ((a << 1) | newbit) & 0x07ff;
}

long int lenumero(char caracter) {
  long int valor = 0;
  int sinal = 1;
  while (caracter != 10) {
    caracter = Serial.read();
    if (caracter >= '0' && caracter <= '9')
      valor = (valor * 10) + (caracter - '0');
    else if (caracter == '-')
      sinal = -1;
    else {
      valor = valor * sinal;
      return (valor);
      sinal = 1;
    }
  }
}

long map2(double v, double in_min, double in_mav, long out_min, long out_mav) {
  return (v - in_min) * (out_mav - out_min) / (in_mav - in_min) + out_min;
}

void enviaSerial() {
  Serial.print(_pulsosencoder[0]);
  Serial.print(',');
  Serial.print(w[0]);
  Serial.print(',');
  Serial.print(_pulsosencoder[1]);
  Serial.print(',');
  Serial.print(w[1]);
  Serial.println();
}

void leSerial() {
  if (Serial.available() > 0) {
    char ch = Serial.read();
    switch (ch) {
      case 'r':
        r = lenumero(ch);
        Q = false;
        break;
      case 'v':
        r = lenumero(ch);
        break;
      case 's':
        s = lenumero(ch);
        Q = true;
        break;
      case 'S':
        S = lenumero(ch);
        Q = true;
        break;
      case 'm':
        m = lenumero(ch);
        break;
      case 'a':
        MA = true;
        break;
      case 'f':
        MA = false;
        break;
    }
  }
}

double leVeloc(int encoderIndex) {
  double theta = double(_pulsosencoder[encoderIndex]) / ENC_RES / 4.0 * 2.0 * PI;
  deltat[encoderIndex] = micros() - tempo[encoderIndex];
  tempo[encoderIndex] = micros();
  double omega = (theta - theta_ant[encoderIndex]) / double(deltat[encoderIndex]) * 1.0e6;
  theta_ant[encoderIndex] = theta;
  return omega;
}

double leang(int encoderIndex) {
  double theta = double(_pulsosencoder[encoderIndex]) * 0.15;
  if (theta > 360) {
    theta = theta - 360;
  } else {
    double theta = double(_pulsosencoder[encoderIndex]) * 0.15;
  }

  return theta;
}

void ContaPulsos(int encoderIndex) {
  _pulsosencoder[encoderIndex]++;
}

void acionaMotor(double v, int motorIndex) {
  v = (v > VFONTE) ? VFONTE : v;
  v = (v < 0) ? 0 : v;
  int V = map2(v, 0, VFONTE, 1, 1024);
  if (motorIndex == 0) {
    Timer1.setPwmDuty(PINO_PWM, V);
  } else if (motorIndex == 1) {
    Timer1.setPwmDuty(PINO_PWM_2, V);
  }
}

void setaEncoders(byte ioMode, byte enablePCI, int pinA, int pinB) {
  switch (NENC) {
    case 1:
      pinMode(pinA, ioMode);
      pinMode(pinB, ioMode);
      break;
    case 2:
      pinMode(pinA, ioMode);
      pinMode(pinB, ioMode);
      pinMode(PINO_A_ENCODER, ioMode);
      pinMode(PINO_B_ENCODER, ioMode);
      break;
  }

  if (enablePCI) {
    PCMSK2 |= MASCARA;
    PCIFR |= 0x04;
    PCICR |= 0x04;
  }

  abOld[0] = abOld[1] = changedCount[0] = changedCount[1] = 0;

  for (byte i = 0; i < NENC; ++i) {
    _pulsosencoder[i] = 0;
  }
}

ISR(PCINT2_vect) {
  for (byte i = 0; i < NENC; ++i) {
    byte abNew = PIND & MASCARA;
    byte changes = abNew ^ abOld[i];
    char wayflag = 2 * abNew ^ abNew;
    byte test2;
    for (byte j = 0; j < 1; ++j) {
      wayflag >>= 2;
      changes >>= 2;
      test2 = changes & 3;
      if (test2 == 1) {
        _pulsosencoder[i]++;
        changedCount[i] = true;
      } else if (test2 == 2) {
        _pulsosencoder[i]--;
        changedCount[i] = true;
      }
    }
    abOld[i] = abNew;
  }
}
