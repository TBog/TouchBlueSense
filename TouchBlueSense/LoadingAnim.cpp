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

        uint8_t const r((mColor >> 16) & 0xFF);
        uint8_t const g((mColor >> 8) & 0xFF);
        uint8_t const b(mColor & 0xFF);
        uint8_t h, s, v;
        uint16_t hue;
        Utils::RgbToHsv(r, g, b, hue, s, v);
        //hue = h * (uint16_t)256;
        if (v < mTailLength)
            v = mTailLength;
    
        int currPixel = mHead;
        Serial.print(F("LoadingAnim "));
        Serial.print(F(" r 0x")); Serial.print(r, HEX);
        Serial.print(F(" g 0x")); Serial.print(g, HEX);
        Serial.print(F(" b 0x")); Serial.print(b, HEX);
        Serial.println();
        for (uint8_t i = 0; i < mTailLength; i += 1)
        {
            Serial.print(F(" #")); Serial.print(i);

            uint32_t color = mColor;
            if (i > 0) {
                uint16_t val = v * (((uint16_t)mTailLength) - i) / mTailLength;
                val = val & 0xFF;
                color = Adafruit_NeoPixel::ColorHSV(hue, s, val);
                
                Serial.print(F(" hue ")); Serial.print(hue, DEC);
                Serial.print(F(" s ")); Serial.print(s, DEC);
                Serial.print(F(" val ")); Serial.print(val, DEC);
            }
    
            Serial.print(F(" | color 0x")); Serial.print(color, HEX);
            Serial.println();
    
            ledStrip.setPixelColor(currPixel, color);
            currPixel = (ledStrip.numPixels() + currPixel - 1) % ledStrip.numPixels();
        }
        for (uint8_t i = mTailLength; i < ledStrip.numPixels(); i += 1)
        {
            ledStrip.setPixelColor(currPixel, 0);
            currPixel = (ledStrip.numPixels() + currPixel - 1) % ledStrip.numPixels();
        }
    }
}
