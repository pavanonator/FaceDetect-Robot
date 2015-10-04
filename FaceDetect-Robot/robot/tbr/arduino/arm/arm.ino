#include <Servo.h>
#include <NewPing.h>
#include <string.h>
#define DEV_ID        2
#define ARM_L1        3
#define ARM_L2        5
#define ARM_L3        6
#define ARM_R1        9
#define ARM_R2        10
#define ARM_R3        11
#define TOP_SWITCH    A1
#define LEFT_TRIGGER  4
#define LEFT_ECHO     7
#define RIGHT_TRIGGER 8
#define RIGHT_ECHO    12
#define MAX_DISTANCE  200

NewPing left_sonar(LEFT_TRIGGER, LEFT_ECHO, MAX_DISTANCE);
NewPing right_sonar(RIGHT_TRIGGER, RIGHT_ECHO, MAX_DISTANCE);
Servo arm_l[3];
Servo arm_r[3];
static int arm_value;

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

bool btmpress() {
  return analogRead(TOP_SWITCH) >= 660;
}

bool toppress() {
  return analogRead(TOP_SWITCH) <= 520;
}

void setarm(int vel) {
  vel = limit(vel, -30, 40);
  if (btmpress()) {
    if (vel < 0) {
      vel = 0;
    }
  }
  if (toppress()) {
    if (vel > 0) {
      vel = 0;
    }
  }
  for (int i = 0; i < 3; i++) {
    arm_l[i].write(vel + 90);
    arm_r[i].write(90 - vel);
  }
}

void setup() {
  arm_l[0].attach(ARM_L1);
  arm_r[0].attach(ARM_R1);
  arm_l[1].attach(ARM_L2);
  arm_r[1].attach(ARM_R2);
  arm_l[2].attach(ARM_L3);
  arm_r[2].attach(ARM_R3);
  pinMode(TOP_SWITCH, INPUT);
  pinMode(13, OUTPUT);
  
  Serial.begin(38400);
  setarm(0);
  digitalWrite(13, HIGH);
  msecs = millis();
}

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
      buf[safesize] = '\0';
    }

    // extract possible message 15 per side
    char *s, *e;
    if ((e = strchr(buf, '\n'))) {
      e[0] = '\0';
      if ((s = strrchr(buf, '['))) {
        // CUSTOMIZE
        sscanf(s, "[%d]", &arm_value);
        arm_value = limit(arm_value, -255, 255);
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }

  setarm(arm_value * 90 / 255);
  
  // 609 539

  if (millis() - msecs > 100) { // 10Hz
    sprintf(msg, "[%d %d %d]",
        DEV_ID, left_sonar.ping() / US_ROUNDTRIP_CM,
        right_sonar.ping() / US_ROUNDTRIP_CM);
    Serial.println(msg);
    msecs = millis();
  }
}
