#include <Servo.h>
#include <string.h>
#define DEV_ID  3
#define CLAW_L  10
#define BOT_L   2
#define TOP_L   3
#define CLAW_R  9
#define BOT_R   0
#define TOP_R   1
#define TOUCH   A0
#define MAX_DISTANCE  200

Servo claw_l;
Servo claw_r;
unsigned long time;
char msg[64];

int limit(int x, int a, int b) {
  if (x > b) {
    return b;
  } else if (x < a) {
    return a;
  } else {
    return x;
  }
}

bool touch() {
  return analogRead(TOUCH) < 3;
}

void setclaw(int vel) {
  vel = limit(vel, -90, 90);
  claw_l.write(90 - vel);
  claw_r.write(vel + 90);
}

void stopclaw() {
  digitalWrite(TOP_R, HIGH);
  digitalWrite(BOT_R, LOW);
  digitalWrite(TOP_L, HIGH);
  digitalWrite(BOT_L, LOW);
  setclaw(0);
  //Serial.println("stopclaw");
}

void openclaw() {
  stopclaw();
  delay(50);
  setclaw(90);
  digitalWrite(BOT_R, HIGH);
  digitalWrite(TOP_R, LOW);
  digitalWrite(BOT_L, HIGH);
  digitalWrite(TOP_L, LOW);
  //Serial.println("openclaw");
}

void closeclaw() {
  stopclaw();
  delay(50);
  setclaw(-90);
  digitalWrite(BOT_R, HIGH);
  digitalWrite(TOP_R, LOW);
  digitalWrite(BOT_L, HIGH);
  digitalWrite(TOP_L, LOW);
  //Serial.println("closeclaw");
}

void setup() {
  claw_l.attach(CLAW_L);
  claw_r.attach(CLAW_R);
  pinMode(TOP_L, OUTPUT);
  pinMode(BOT_L, OUTPUT);
  pinMode(TOP_R, OUTPUT);
  pinMode(BOT_R, OUTPUT);
  pinMode(TOUCH, INPUT);
  pinMode(11, OUTPUT);
  
  Serial.begin(38400);
  stopclaw();
  digitalWrite(11, HIGH);
  time = millis();
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
      case ' ':
        stopclaw();
        break;
      case 'o':
        openclaw();
        break;
      case 'k':
        closeclaw();
        break;
      case '\n':
        // do nothing
        break;
      default:
        //Serial.println("err: bad command");
        //Serial.println(c);
        break;
    }
  }
  if (millis() - time > 100) { // 10Hz
    sprintf(msg, "[%d %d]", DEV_ID,
        touch());
    Serial.println(msg);
    time = millis();
  }
}
