#ifndef __UTILS_H__
#define __UTILS_H__

#include "Arduino.h"

namespace Utils
{

    // This function is only an approximation.
    void rgb2hsv_approximate(uint8_t r, uint8_t g, uint8_t b, uint8_t &h, uint8_t &s, uint8_t &v);

} // end namespace Utils

#endif //__UTILS_H__