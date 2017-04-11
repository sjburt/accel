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

#define BUZZER 4

#define ACC_X 0
#define ACC_Y 1
#define ACC_Z 2

#define DEADBAND 64
#define CENTER 512

#define AXIS ACC_X
#define NUM_AVES 20

int xrdg[NUM_AVES], yrdg[NUM_AVES], zrdg[NUM_AVES];

// definitions
void left();
void right();
void stop();


void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(ENABLE_MOTOR, OUTPUT);
  stop();
  Serial.begin(115200);

  memset(xrdg, 0, sizeof(xrdg));
}


void loop() {
  int x = analogRead(ACC_X);

  if (x > CENTER+DEADBAND) {
    left();
  } else if (x < CENTER-DEADBAND) {
    right();
  } else {
    stop();
  }

  Serial.print(x, DEC);
  Serial.print("\n");

  delay(10);
}


// ========================
void left() {
  digitalWrite(MOTOR_DIR, LOW);
  digitalWrite(ENABLE_MOTOR, HIGH);
}

void right() {
  digitalWrite(MOTOR_DIR, HIGH);
  digitalWrite(ENABLE_MOTOR, HIGH);
}

void stop() {
  digitalWrite(ENABLE_MOTOR, LOW);
}
