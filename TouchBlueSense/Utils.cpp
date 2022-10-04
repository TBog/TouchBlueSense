#include "Utils.h"
#include "lib8tion.h"

#define HUE_RED      0
#define HUE_ORANGE  32
#define HUE_YELLOW  64
#define HUE_GREEN   96
#define HUE_AQUA   128
#define HUE_BLUE   160
#define HUE_PURPLE 192
#define HUE_PINK   224
#define FIXFRAC8(N,D) (((N)*256)/(D))

// source: https://github.com/FastLED/FastLED/blob/master/src/hsv2rgb.h
void Utils::rgb2hsv_approximate(uint8_t r, uint8_t g, uint8_t b, uint8_t &h, uint8_t &s, uint8_t &v)
{
    // find desaturation
    uint8_t desat = 255;
    if (r < desat)
        desat = r;
    if (g < desat)
        desat = g;
    if (b < desat)
        desat = b;

    // remove saturation from all channels
    r -= desat;
    g -= desat;
    b -= desat;

    // Serial.print("desat="); Serial.print(desat); Serial.println("");

    // uint8_t orig_desat = sqrt16( desat * 256);
    // Serial.print("orig_desat="); Serial.print(orig_desat); Serial.println("");

    // saturation is opposite of desaturation
    s = 255 - desat;
    // Serial.print("s.1="); Serial.print(s); Serial.println("");

    if (s != 255)
    {
        // undo 'dimming' of saturation
        s = 255 - sqrt16((255 - s) * 256);
    }
    // without lib8tion: float ... ew ... sqrt... double ew, or rather, ew ^ 0.5
    // if( s != 255 ) s = (255 - (256.0 * sqrt( (float)(255-s) / 256.0)));
    // Serial.print("s.2="); Serial.print(s); Serial.println("");

    // at least one channel is now zero
    // if all three channels are zero, we had a
    // shade of gray.
    if ((r + g + b) == 0)
    {
        // we pick hue zero for no special reason
        h = 0;
        v = 255 - s;
        s = 0;
        return;
    }

    // scale all channels up to compensate for desaturation
    if (s < 255)
    {
        if (s == 0)
            s = 1;
        uint32_t scaleup = 65535 / (s);
        r = ((uint32_t)(r)*scaleup) / 256;
        g = ((uint32_t)(g)*scaleup) / 256;
        b = ((uint32_t)(b)*scaleup) / 256;
    }
    // Serial.print("r.2="); Serial.print(r); Serial.println("");
    // Serial.print("g.2="); Serial.print(g); Serial.println("");
    // Serial.print("b.2="); Serial.print(b); Serial.println("");

    uint16_t total = r + g + b;

    // Serial.print("total="); Serial.print(total); Serial.println("");

    // scale all channels up to compensate for low values
    if (total < 255)
    {
        if (total == 0)
            total = 1;
        uint32_t scaleup = 65535 / (total);
        r = ((uint32_t)(r)*scaleup) / 256;
        g = ((uint32_t)(g)*scaleup) / 256;
        b = ((uint32_t)(b)*scaleup) / 256;
    }
    // Serial.print("r.3="); Serial.print(r); Serial.println("");
    // Serial.print("g.3="); Serial.print(g); Serial.println("");
    // Serial.print("b.3="); Serial.print(b); Serial.println("");

    if (total > 255)
    {
        v = 255;
    }
    else
    {
        v = qadd8(desat, total);
        // undo 'dimming' of brightness
        if (v != 255)
            v = sqrt16(v * 256);
        // without lib8tion: float ... ew ... sqrt... double ew, or rather, ew ^ 0.5
        // if( v != 255) v = (256.0 * sqrt( (float)(v) / 256.0));
    }

    // Serial.print("v="); Serial.print(v); Serial.println("");

#if 0
    
    //#else
    if( v != 255) {
        // this part could probably use refinement/rethinking,
        // (but it doesn't overflow & wrap anymore)
        uint16_t s16;
        s16 = (s * 256);
        s16 /= v;
        //Serial.print("s16="); Serial.print(s16); Serial.println("");
        if( s16 < 256) {
            s = s16;
        } else {
            s = 255; // clamp to prevent overflow
        }
    }
#endif

    // Serial.print("s.3="); Serial.print(s); Serial.println("");

    // since this wasn't a pure shade of gray,
    // the interesting question is what hue is it

    // start with which channel is highest
    // (ties don't matter)
    uint8_t highest = r;
    if (g > highest)
        highest = g;
    if (b > highest)
        highest = b;

    if (highest == r)
    {
        // Red is highest.
        // Hue could be Purple/Pink-Red,Red-Orange,Orange-Yellow
        if (g == 0)
        {
            // if green is zero, we're in Purple/Pink-Red
            h = (HUE_PURPLE + HUE_PINK) / 2;
            h += scale8(qsub8(r, 128), FIXFRAC8(48, 128));
        }
        else if ((r - g) > g)
        {
            // if R-G > G then we're in Red-Orange
            h = HUE_RED;
            h += scale8(g, FIXFRAC8(32, 85));
        }
        else
        {
            // R-G < G, we're in Orange-Yellow
            h = HUE_ORANGE;
            h += scale8(qsub8((g - 85) + (171 - r), 4), FIXFRAC8(32, 85)); // 221
        }
    }
    else if (highest == g)
    {
        // Green is highest
        // Hue could be Yellow-Green, Green-Aqua
        if (b == 0)
        {
            // if Blue is zero, we're in Yellow-Green
            //   G = 171..255
            //   R = 171..  0
            h = HUE_YELLOW;
            uint8_t radj = scale8(qsub8(171, r), 47); // 171..0 -> 0..171 -> 0..31
            uint8_t gadj = scale8(qsub8(g, 171), 96); // 171..255 -> 0..84 -> 0..31;
            uint8_t rgadj = radj + gadj;
            uint8_t hueadv = rgadj / 2;
            h += hueadv;
            // h += scale8( qadd8( 4, qadd8((g - 128), (128 - r))),
            //              FIXFRAC8(32,255)); //
        }
        else
        {
            // if Blue is nonzero we're in Green-Aqua
            if ((g - b) > b)
            {
                h = HUE_GREEN;
                h += scale8(b, FIXFRAC8(32, 85));
            }
            else
            {
                h = HUE_AQUA;
                h += scale8(qsub8(b, 85), FIXFRAC8(8, 42));
            }
        }
    }
    else /* highest == b */
    {
        // Blue is highest
        // Hue could be Aqua/Blue-Blue, Blue-Purple, Purple-Pink
        if (r == 0)
        {
            // if red is zero, we're in Aqua/Blue-Blue
            h = HUE_AQUA + ((HUE_BLUE - HUE_AQUA) / 4);
            h += scale8(qsub8(b, 128), FIXFRAC8(24, 128));
        }
        else if ((b - r) > r)
        {
            // B-R > R, we're in Blue-Purple
            h = HUE_BLUE;
            h += scale8(r, FIXFRAC8(32, 85));
        }
        else
        {
            // B-R < R, we're in Purple-Pink
            h = HUE_PURPLE;
            h += scale8(qsub8(r, 85), FIXFRAC8(32, 85));
        }
    }

    h += 1;
}

void Utils::HsvToRgb(uint16_t h, uint8_t s, uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b)
{
    unsigned char region, remainder, p, q, t;
    
    if (s == 0)
    {
        r = v;
        g = v;
        b = v;
        return;
    }
    
    // Remap 0-65535 to 0-1529. Pure red is CENTERED on the 64K rollover;
    // 0 is not the start of pure red, but the midpoint...a few values above
    // zero and a few below 65536 all yield pure red (similarly, 32768 is the
    // midpoint, not start, of pure cyan). The 8-bit RGB hexcone (256 values
    // each for red, green, blue) really only allows for 1530 distinct hues
    // (not 1536, more on that below), but the full unsigned 16-bit type was
    // chosen for hue so that one's code can easily handle a contiguous color
    // wheel by allowing hue to roll over in either direction.
    h = (h * 1530L + 32768) / 65536;

    region = h / 255;//region = h / 43;
    remainder = (h - (region * 255)) * 6;//remainder = (h - (region * 43)) * 6;
    
    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
    
    switch (region)
    {
        case 0:
            r = v; g = t; b = p;
            break;
        case 1:
            r = q; g = v; b = p;
            break;
        case 2:
            r = p; g = v; b = t;
            break;
        case 3:
            r = p; g = q; b = v;
            break;
        case 4:
            r = t; g = p; b = v;
            break;
        default:
            r = v; g = p; b = q;
            break;
    }
}

void Utils::RgbToHsv(uint8_t r, uint8_t g, uint8_t b, uint16_t &h, uint8_t &s, uint8_t &v)
{
    uint8_t rgbMin, rgbMax;

    rgbMin = r < g ? (r < b ? r : b) : (g < b ? g : b);
    rgbMax = r > g ? (r > b ? r : b) : (g > b ? g : b);
    
    v = rgbMax;
    if (v == 0)
    {
        h = 0;
        s = 0;
        return;
    }

    s = 255 * long(rgbMax - rgbMin) / v;
    if (s == 0)
    {
        h = 0;
        return;
    }

    if (rgbMax == r)
        h = 0 + 255 * (g - b) / (rgbMax - rgbMin);//h = 0 + 43 * (g - b) / (rgbMax - rgbMin);
    else if (rgbMax == g)
        h = 510 + 255 * (b - r) / (rgbMax - rgbMin);//h = 85 + 43 * (b - r) / (rgbMax - rgbMin);
    else
        h = 1020 + 255 * (r - g) / (rgbMax - rgbMin);//h = 171 + 43 * (r - g) / (rgbMax - rgbMin);
}