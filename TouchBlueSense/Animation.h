#ifndef _ANIMATION_H_
#define _ANIMATION_H_

#include "Arduino.h"

class Adafruit_NeoPixel;

class Animation
{
public:
    Animation(){};

    virtual void update(time_t deltaTime, Adafruit_NeoPixel &ledStrip){};
};

#endif //_ANIMATION_H_
