#include <Servo.h>
#include <NewPing.h>
#include <string.h>
#define DEV_ID        1
#define WHEEL_R1      3
#define WHEEL_R2      5
#define WHEEL_R3      6
#define WHEEL_L1      9
#define WHEEL_L2      10
#define WHEEL_L3      11
#define TRIGGER_PIN   7
#define ECHO_PIN      8
#define MAX_DISTANCE  200

NewPing back_sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo wheel_l[3];
Servo wheel_r[3];
static int left_value, right_value;

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

void setwheels(int left, int right) {
  left = -limit(left, -90, 90);
  right = limit(right, -90, 90);
  wheel_l[0].write(left + 90);
  wheel_l[1].write(90 - left);
  wheel_l[2].write(left + 90);
  wheel_r[0].write(right + 90);
  wheel_r[1].write(90 - right);
  wheel_r[2].write(right + 90);
}

void setup() {
  wheel_l[0].attach(WHEEL_L1);
  wheel_r[0].attach(WHEEL_R1);
  wheel_l[1].attach(WHEEL_L2);
  wheel_r[1].attach(WHEEL_R2);
  wheel_l[2].attach(WHEEL_L3);
  wheel_r[2].attach(WHEEL_R3);
  pinMode(13, OUTPUT);

  Serial.begin(38400);
  setwheels(0, 0);
  digitalWrite(13, HIGH);
  delay(50);
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

    // extract possible message 688 bottom, 615 top
    char *s, *e;
    if ((e = strchr(buf, '\n'))) {
      e[0] = '\0';
      if ((s = strrchr(buf, '['))) {
        // CUSTOMIZE
        sscanf(s, "[%d %d]", &left_value, &right_value);
        left_value = limit(left_value, -255, 255);
        right_value = limit(right_value, -255, 255);
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }

  setwheels(left_value * 90 / 255, right_value * 90 / 255);

  if (millis() - msecs > 100) { // 10Hz
    sprintf(msg, "[%d %d]", DEV_ID,
        back_sonar.ping() / US_ROUNDTRIP_CM);
    Serial.println(msg);
    msecs = millis();
  }
}
