#include "BlinkAnim.h"
#include "Adafruit_NeoPixel.h"

BlinkAnim::BlinkAnim(uint32_t color, time_t timeOn, time_t timeOff)
    : mTimer(timeOff), mTimeOn(timeOn), mTimeOff(timeOff), mColor(color), mIsOn(false)
{
    // we have mTimer=timeOff and mIsOn=false so that on the first update it will turn on
}

void BlinkAnim::update(time_t deltaTime, Adafruit_NeoPixel &ledStrip)
{
    mTimer += deltaTime;
    if (mIsOn)
    {
        if (mTimer > mTimeOn)
        {
            mTimer -= mTimeOn;
            mIsOn = false;
            ledStrip.clear();
        }
    }
    else
    {
        if (mTimer > mTimeOff)
        {
            mTimer -= mTimeOff;
            mIsOn = true;
            ledStrip.fill(mColor);
        }
    }
}
