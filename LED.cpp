#include "LED.h"

const int LED_pin[LED_COUNT] = {RED, GREEN, BLUE};

void LEDClass::init(){
  for(int i = 0; i < LED_COUNT; i++){
    pinMode(LED_pin[i], OUTPUT);
    digitalWrite(LED_pin[i], LOW);
  }
}

void LEDClass::setAll(bool is_on){
  for(int i = 0; i < LED_COUNT; i++){
    digitalWrite(LED_pin[i], is_on);
  }
}

void LEDClass::chase(int count){
  for(int i = 0; i < count; i++){
    for(int j = 0; j < LED_COUNT; j++){
      digitalWrite(LED_pin[j], HIGH);
      delay(25);
      digitalWrite(LED_pin[j], LOW);
    }
  }
}

void LEDClass::blink(COLOR color, int count){
  for(int i = 0; i < count; i++){
    digitalWrite((int)color, HIGH);
    delay(25);
    digitalWrite((int)color, LOW);
    delay(25);
  }
}

void LEDClass::set(COLOR color, bool is_high){
  digitalWrite((int)color, is_high);
}
