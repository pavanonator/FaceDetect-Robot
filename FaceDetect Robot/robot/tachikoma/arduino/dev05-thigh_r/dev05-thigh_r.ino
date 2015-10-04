#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <string.h>

#define DEV_ID 5
#define LIN_PIN1  3
#define LIN_PIN2  4
#define LOUT_PIN1 5
#define LOUT_PIN2 6
#define RIN_PIN1  7
#define RIN_PIN2  8
#define ROUT_PIN1 11
#define ROUT_PIN2 12

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motors[4];
static int v[2];
static char signature;
static char leftsig;
static char rightsig;
static char instr_activate;

const int bufsize = 256;
const int safesize = bufsize / 2;
char buf[bufsize];
char msg[bufsize];
char wbuf[safesize];
unsigned long msecs;
char numbuf[4];

int limit(int x, int a, int b) {
  if (x > b) {
    return b;
  } else if (x < a) {
    return a;
  } else {
    return x;
  }
}

void setmotors(int leftv, int rightv) {
  bool leftisneg = leftv < 0;
  bool rightisneg = rightv < 0;
  leftv = limit(abs(leftv), 0, 128);
  rightv = limit(abs(rightv), 0, 128);
  if (leftsig == signature) {
    motors[0]->setSpeed(leftv);
    motors[1]->setSpeed(leftv);
    if (leftisneg) {
      motors[0]->run(BACKWARD);
      motors[1]->run(FORWARD);
    } else {
      motors[0]->run(FORWARD);
      motors[1]->run(BACKWARD);
    }
  } else {
    motors[0]->setSpeed(0);
    motors[1]->setSpeed(0);
  }
  if (rightsig == signature) {
    motors[2]->setSpeed(rightv);
    motors[3]->setSpeed(rightv);
    if (rightisneg) {
      motors[2]->run(FORWARD);
      motors[3]->run(BACKWARD);
    } else {
      motors[2]->run(BACKWARD);
      motors[3]->run(FORWARD);
    }
  } else {
    motors[2]->setSpeed(0);
    motors[3]->setSpeed(0);
  }
}

void read_signatures() {
  char leftbit0 = (digitalRead(LIN_PIN1) == HIGH);
  char leftbit1 = (digitalRead(LIN_PIN2) == HIGH) << 1;
  leftsig = leftbit1 | leftbit0;
  char rightbit0 = (digitalRead(RIN_PIN1) == HIGH);
  char rightbit1 = (digitalRead(RIN_PIN2) == HIGH) << 1;
  rightsig = rightbit1 | rightbit0;
}

void write_signature() {
  char bit0 = (signature & 0x01);
  char bit1 = (signature & 0x02) >> 1;
  digitalWrite(LOUT_PIN1, bit0 ? HIGH : LOW);
  digitalWrite(LOUT_PIN2, bit1 ? HIGH : LOW);
  digitalWrite(ROUT_PIN1, bit0 ? HIGH : LOW);
  digitalWrite(ROUT_PIN2, bit1 ? HIGH : LOW);
}

void setup() {
  motors[0] = AFMS.getMotor(1);
  motors[1] = AFMS.getMotor(2);
  motors[2] = AFMS.getMotor(3);
  motors[3] = AFMS.getMotor(4);

  pinMode(A0, INPUT);
  
  // signature synchronization
  pinMode(LIN_PIN1, INPUT);
  pinMode(LIN_PIN2, INPUT);
  pinMode(LOUT_PIN1, OUTPUT);
  pinMode(LOUT_PIN2, OUTPUT);
  pinMode(RIN_PIN1, INPUT);
  pinMode(RIN_PIN2, INPUT);
  pinMode(ROUT_PIN1, OUTPUT);
  pinMode(ROUT_PIN2, OUTPUT);
  write_signature();

  pinMode(13, OUTPUT); // set status LED to OUTPUT and HIGH
  digitalWrite(13, HIGH);

  AFMS.begin();
  setmotors(0, 0);
  Serial.begin(57600);
  msecs = millis();
}

static int targetv[2];
static int prevv[2];
static int targetp[2];

void loop() {
  int nbytes = 0;
  if ((nbytes = Serial.available())) {
    // read + attach null byte
    int obytes = strlen(buf);
    Serial.readBytes(&buf[obytes], nbytes);
    buf[nbytes + obytes] = '\0';

    // resize just in case
    if (strlen(buf) > safesize) {
      memmove(buf, &buf[strlen(buf) - safesize], safesize);
      buf[safesize] = '\0'; // just in case
    }

    // extract possible message
    char *s, *e;
    if ((e = strchr(buf, '\n'))) {
      e[0] = '\0';
      if ((s = strrchr(buf, '['))) {
        // CUSTOMIZE
        sscanf(s, "[%c %c %d %d %d %d]\n",
          &instr_activate,
          &signature,
          &targetp[0],
          &targetp[1],
          &targetv[0],
          &targetv[1]);
        write_signature();
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }
  int deltav[2] = { targetv[0] - prevv[0], targetv[1] - prevv[1] };
  int sign[2] = { deltav[0] >= 0 ? 1 : -1, deltav[1] >= 0 ? 1 : -1 };
  deltav[0] *= sign[0];
  deltav[1] *= sign[1];
  if (deltav[0] > 4) {
    deltav[0] = 4;
  }
  if (deltav[1] > 4) {
    deltav[1] = 4;
  }
  v[0] = limit(prevv[0] + (deltav[0] * sign[0]), -255, 255);
  v[1] = limit(prevv[1] + (deltav[1] * sign[1]), -255, 255);
  read_signatures();
  setmotors(v[0], v[1]);
  prevv[0] = v[0];
  prevv[1] = v[1];

  if (millis() - msecs > 100) {
    sprintf(wbuf, "[%d %d %d %d]\n",
      DEV_ID,
      analogRead(A0),
      v[0],
      v[1]);
    Serial.print(wbuf);
    msecs = millis();
  }
}
