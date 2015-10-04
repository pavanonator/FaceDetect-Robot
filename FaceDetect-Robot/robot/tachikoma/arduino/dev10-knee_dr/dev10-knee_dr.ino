#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <string.h>

#define DEV_ID 10

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motors[4];
static int v;
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

void setmotors(int v) {
  bool isneg = v < 0;
  v = limit(abs(v), 0, 128);
  motors[0]->setSpeed(v);
  motors[1]->setSpeed(v);
  motors[2]->setSpeed(v);
  motors[3]->setSpeed(v);
  if (isneg) {
    motors[0]->run(BACKWARD);
    motors[1]->run(FORWARD);
    motors[2]->run(FORWARD);
    motors[3]->run(BACKWARD);
  } else {
    motors[0]->run(FORWARD);
    motors[1]->run(BACKWARD);
    motors[2]->run(BACKWARD);
    motors[3]->run(FORWARD);
  }
}

void setup() {
  motors[0] = AFMS.getMotor(1);
  motors[1] = AFMS.getMotor(2);
  motors[2] = AFMS.getMotor(3);
  motors[3] = AFMS.getMotor(4);

  pinMode(A0, INPUT);

  pinMode(13, OUTPUT); // set status LED to OUTPUT and HIGH
  digitalWrite(13, HIGH);

  AFMS.begin();
  setmotors(0);
  Serial.begin(57600);
  msecs = millis();
}

static int targetv;
static int prevv;
static int targetp;

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
        sscanf(s, "[%c %d %d]\n",
          &instr_activate,
          &targetp,
          &targetv);
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }
  int deltav = targetv - prevv;
  int sign = deltav >= 0 ? 1 : -1;
  deltav *= sign;
  if (deltav > 4) {
    deltav = 4;
  }
  v = prevv + (deltav * sign);
  v = limit(v, -255, 255);
  setmotors(v);
  prevv = v;

  if (millis() - msecs > 100) {
    sprintf(wbuf, "[%d %d %d]\n",
      DEV_ID,
      analogRead(A0),
      v);
    Serial.print(wbuf);
    msecs = millis();
  }
}
