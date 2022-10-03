#ifndef _LOADING_ANIM_H_
#define _LOADING_ANIM_H_

#include "Animation.h"

class LoadingAnim : public Animation
{
    time_t mTimer;
    time_t mDelay;
    uint32_t mColor;
    uint8_t mTailLength;
    uint8_t mHead;

public:
    LoadingAnim(uint32_t color, uint8_t tailLength = 5, time_t delay = 33);

    virtual void update(time_t deltaTime, Adafruit_NeoPixel &ledStrip) override;
};

#endif //_LOADING_ANIM_H_
