/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#define ENABLE_MOTOR 10
#define MOTOR_DIR 12

#define ENABLE_LIGHT 11
#define LIGHT_DIR 13


#define BUZZER 4

#define ACC_X 0
#define ACC_Y 1
#define ACC_Z 2

#define LEFT_STOP 2
#define L_GND 3
#define RIGHT_STOP 6
#define R_GND 7

#define DEADBAND 8
#define AXIS ACC_X
#define MOT_POWER 255    // Between 0 and 255.



#define NUM_AVES 20

void left(void), right(void), stop(void);

int left_stop(void), right_stop(void);

struct aves_t {
  int ptr;
  int sum;
  int values[NUM_AVES];
};

struct aves_t xrdg, yrdg, zrdg;
int center;

void init_aves(struct aves_t * rdgs) {
  memset(rdgs, 0, sizeof(*rdgs));
}

int rolling_ave(int rdg, struct aves_t *rdgs, int n) {
  rdgs->sum += rdg - rdgs->values[rdgs->ptr];
  rdgs->values[rdgs->ptr] = rdg;
  rdgs->ptr++;
  rdgs->ptr %= NUM_AVES;

  return rdgs->sum / NUM_AVES;
}


void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(ENABLE_MOTOR, OUTPUT);

  pinMode(LIGHT_DIR, OUTPUT);
  pinMode(ENABLE_LIGHT, OUTPUT);

  digitalWrite(LIGHT_DIR, LOW);

  pinMode(LEFT_STOP, INPUT_PULLUP);
  pinMode(RIGHT_STOP, INPUT_PULLUP);
  pinMode(L_GND, OUTPUT);
  pinMode(R_GND, OUTPUT);

  digitalWrite(L_GND, LOW);
  digitalWrite(R_GND, LOW);


  stop();
  Serial.begin(115200);

  init_aves(&xrdg);
  int x, xave;
  for (int i = 0; i < NUM_AVES*2; i++) {
    x = analogRead(ACC_X);
    xave = rolling_ave(x, &xrdg, NUM_AVES);
  }

  center = xave;

  Serial.print(center, DEC);
  Serial.print("\n");
}


void loop() {
  int x = analogRead(ACC_X);
  int xave = rolling_ave(x, &xrdg, NUM_AVES);
  static int q, p;

  if (xave > center+DEADBAND) {
    left();
  } else if (xave < center-DEADBAND) {
    right();
  } else {
    stop();
  }
  q++;
  if (q%5 == 0) {
    Serial.print(" ");
    Serial.print(p, DEC);
    Serial.print(" ");
    Serial.print(left_stop(), DEC);
    Serial.print(" ");
    Serial.print(right_stop(), DEC);
    Serial.print(" ");
    Serial.print(x, DEC);
    Serial.print(" ");
    Serial.print(xave, DEC);
    Serial.print("         \r");
  }

  if (q % 50 == 0) {
    if (p) {
      p = 0;
      digitalWrite(ENABLE_LIGHT, LOW);
    } else {
      p = 1;
      digitalWrite(ENABLE_LIGHT, HIGH);
    }
  }

  delay(10);
}


// ========================
int left_stop() {
  return !digitalRead(LEFT_STOP);
}

int right_stop() {
  return !digitalRead(RIGHT_STOP);
}

void left() {
  if (!left_stop()) {
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(ENABLE_MOTOR, MOT_POWER);
  } else stop();
}

void right() {
  if (!right_stop()) {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(ENABLE_MOTOR, MOT_POWER);
  } else stop();
}

void stop() {
  digitalWrite(ENABLE_MOTOR, LOW);
}
