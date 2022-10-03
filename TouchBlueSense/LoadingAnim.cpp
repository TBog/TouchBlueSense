#include "LoadingAnim.h"
#include "Adafruit_NeoPixel.h"
#include "Utils.h"

LoadingAnim::LoadingAnim(uint32_t color, uint8_t tailLength, time_t delay)
    : mTimer(delay), mDelay(delay), mColor(color), mTailLength(tailLength), mHead(0)
{
    // we have mTimer=delay so that on the first update it will move
}

void LoadingAnim::update(time_t deltaTime, Adafruit_NeoPixel &ledStrip)
{
    mTimer += deltaTime;
    if (mTimer > mDelay)
    {
        mTimer -= mDelay;
        mHead = (mHead + 1) % ledStrip.numPixels();
    }

    uint8_t const r((mColor >> 16) & 0xFF);
    uint8_t const g((mColor >> 8) & 0xFF);
    uint8_t const b(mColor & 0xFF);
    uint8_t h, s, v;
    Utils::rgb2hsv_approximate(r, g, b, h, s, v);
    uint16_t hue = h * (uint16_t)256;
    if (v < mTailLength)
        v = mTailLength;

    int currPixel = mHead;
    for (int i = 0; i < mTailLength; i += 1)
    {
        uint8_t val = v / mTailLength * (mTailLength - i);
        const auto color = i > 0 ? Adafruit_NeoPixel::ColorHSV(hue, s, v) : mColor;
        ledStrip.setPixelColor(currPixel, color);
        currPixel = (currPixel + 1) % ledStrip.numPixels();
    }
    for (int i = mTailLength; i < ledStrip.numPixels(); i += 1)
    {
        ledStrip.setPixelColor(currPixel, 0);
        currPixel = (currPixel + 1) % ledStrip.numPixels();
    }
}
