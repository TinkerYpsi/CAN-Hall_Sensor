// LED.h
#ifndef _LED_h
#define _LED_h

#define LED_COUNT 3

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

typedef enum _color {
  RED = 6,
  GREEN = 5,
  BLUE = 4
} COLOR;

class LEDClass {

  public:
    static void init();
    static void setAll(bool is_on);
    static void chase(int count);
    static void blink(COLOR color, int count);
    static void set(COLOR color, bool is_high);

  private:
};

extern LEDClass LED;
#endif


