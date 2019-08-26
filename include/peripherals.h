#pragma once

#define LED_RED 13
#define LED_BLUE 12
#define BUZZER 21
#define PIN_STATE 15

bool condition_red = false;
bool condition_blue = false;

void initializePeripherals() {
  // LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  
  // Buzzer/sounder
  pinMode(BUZZER, OUTPUT);

  // State/debug pin
  pinMode(PIN_STATE, OUTPUT);

  // Set state pin to low by default
  delay(10);
  digitalWrite(PIN_STATE, LOW);
}

void red(bool condition = true) {
  condition_red = condition;

  digitalWrite(LED_RED, condition ? HIGH : LOW);
}

void blue(bool condition = true) {
  condition_blue = condition;

  digitalWrite(LED_BLUE, condition ? HIGH : LOW);
}

void redToggle() {
  red(!condition_red);
}

void blueToggle() {
  blue(!condition_blue);
}

void ring(int times = 1, int intensity = 5, int delay_ms = 50) {
  // intensity: 5 = LOW; 50 = HIGH

  for(int r=0; r<times; r++) {
    delay(delay_ms);
    digitalWrite(BUZZER, HIGH);

    delay(intensity);
    digitalWrite(BUZZER, LOW);
  }
}

void ringMicroseconds(int times = 1, int intensity = 50, int delay_us = 10) {
  for(int r=0; r<times; r++) {
    delayMicroseconds(delay_us);
    digitalWrite(BUZZER, HIGH);

    delayMicroseconds(intensity);
    digitalWrite(BUZZER, LOW);
  }
}

void state(bool condition = true) {
  digitalWrite(PIN_STATE, condition ? HIGH : LOW);
}
