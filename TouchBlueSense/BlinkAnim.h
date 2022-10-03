#ifndef _BLINK_ANIM_H_
#define _BLINK_ANIM_H_

#include "Animation.h"

class BlinkAnim : public Animation
{
    time_t mTimer;
    time_t mTimeOn;
    time_t mTimeOff;
    uint32_t mColor;
    bool mIsOn;

public:
    BlinkAnim(uint32_t color, time_t timeOn = 60, time_t timeOff = 50);

    virtual void update(time_t deltaTime, Adafruit_NeoPixel &ledStrip) override;
};

#endif //_BLINK_ANIM_H_
