#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <string.h>

#define DEV_ID 2

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motors[4];
static int v[2];
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

void setmotors(int topv, int btmv) {
  bool topisneg = topv < 0;
  bool btmisneg = btmv < 0;
  topv = limit(abs(topv), 0, 128);
  btmv = limit(abs(btmv), 0, 128);
  motors[0]->setSpeed(topv);
  motors[1]->setSpeed(topv);
  motors[2]->setSpeed(btmv);
  motors[3]->setSpeed(btmv);
  if (topisneg) {
    motors[0]->run(BACKWARD);
    motors[1]->run(FORWARD);
  } else {
    motors[0]->run(FORWARD);
    motors[1]->run(BACKWARD);
  }
  if (btmisneg) {
    motors[2]->run(BACKWARD);
    motors[3]->run(FORWARD);
  } else {
    motors[2]->run(FORWARD);
    motors[3]->run(BACKWARD);
  }
}

void setup() {
  motors[0] = AFMS.getMotor(1);
  motors[1] = AFMS.getMotor(2);
  motors[2] = AFMS.getMotor(3);
  motors[3] = AFMS.getMotor(4);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

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
        sscanf(s, "[%c %d %d %d %d]\n",
          &instr_activate,
          &targetp[0],
          &targetp[1],
          &targetv[0],
          &targetv[1]);
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
  setmotors(v[0], v[1]);
  prevv[0] = v[0];
  prevv[1] = v[1];

  if (millis() - msecs > 100) {
    sprintf(wbuf, "[%d %d %d %d %d]\n",
      DEV_ID,
      analogRead(A0),
      analogRead(A1),
      v[0],
      v[1]);
    Serial.print(wbuf);
    msecs = millis();
  }
}
