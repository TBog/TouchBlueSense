// Fake FastLED.h so I don't have to change all the files I get from there

#ifndef FASTLED_NAMESPACE_BEGIN
#define FASTLED_NAMESPACE_BEGIN
#define FASTLED_NAMESPACE_END
#define FASTLED_USING_NAMESPACE
#endif

#define __INC_LED_SYSDEFS_H

#ifndef __INC_FASTLED_CONFIG_H
#define __INC_FASTLED_CONFIG_H

///@file fastled_config.h
/// contains definitions that can be used to configure FastLED at compile time

// Use this option only for debugging pin access and forcing software pin access.  Note that
// software pin access only works in Arduino based environments.  Forces use of digitalWrite
// methods for pin access vs. direct hardware port access
// #define FASTLED_FORCE_SOFTWARE_PINS

// Use this option only for debugging bitbang'd spi access or to work around bugs in hardware
// spi access.  Forces use of bit-banged spi, even on pins that has hardware SPI available.
// #define FASTLED_FORCE_SOFTWARE_SPI

// Use this to force FastLED to allow interrupts in the clockless chipsets (or to force it to
// disallow), overriding the default on platforms that support this.  Set the value to 1 to
// allow interrupts or 0 to disallow them.
// #define FASTLED_ALLOW_INTERRUPTS 1
// #define FASTLED_ALLOW_INTERRUPTS 0

// Use this to allow some integer overflows/underflows in the inoise functions.
// The original implementions allowed this, and had some discontinuties in the noise
// output.  It's technically an implementation bug, and was fixed, but you may wish
// to preserve the old look and feel of the inoise functions in your existing animations.
// The default is 0:  NO overflow, and 'continuous' noise output, aka the fixed way.
// #define FASTLED_NOISE_ALLOW_AVERAGE_TO_OVERFLOW 0
// #define FASTLED_NOISE_ALLOW_AVERAGE_TO_OVERFLOW 1

// Use this toggle whether or not to use the 'fixed' FastLED scale8.  The initial scale8
// had a problem where scale8(255,255) would give you 254.  This is now fixed, and that
// fix is enabled by default.  However, if for some reason you have code that is not
// working right as a result of this (e.g. code that was expecting the old scale8 behavior)
// you can disable it here.
#define FASTLED_SCALE8_FIXED 1
// #define FASTLED_SCALE8_FIXED 0

// Use this toggle whether to use 'fixed' FastLED pixel blending, including ColorFromPalette.
// The prior pixel blend functions had integer-rounding math errors that led to
// small errors being inadvertently added to the low bits of blended colors, including colors
// retrieved from color palettes using LINEAR_BLEND.  This is now fixed, and the
// fix is enabled by default.  However, if for some reason you wish to run with the old
// blending, including the integer rounding and color errors, you can disable the bugfix here.
#define FASTLED_BLEND_FIXED 1
// #define FASTLED_BLEND_FIXED 0

// Use this toggle whether to use 'fixed' FastLED 8- and 16-bit noise functions.
// The prior noise functions had some math errors that led to 'discontinuities' in the
// output, which by definition should be smooth and continuous.  The bug led to 
// noise function output that had 'edges' and glitches in it.  This is now fixed, and the
// fix is enabled by default.  However, if for some reason you wish to run with the old
// noise code, including the glitches, you can disable the bugfix here.
#define FASTLED_NOISE_FIXED 1
//#define FASTLED_NOISE_FIXED 0

// Use this to determine how many times FastLED will attempt to re-transmit a frame if interrupted
// for too long by interrupts.
#ifndef FASTLED_INTERRUPT_RETRY_COUNT
#define FASTLED_INTERRUPT_RETRY_COUNT 2
#endif

// Use this toggle to enable global brightness in contollers that support is (ADA102 and SK9822).
// It changes how color scaling works and uses global brightness before scaling down color values.
// This enable much more accurate color control on low brightness settings.
//#define FASTLED_USE_GLOBAL_BRIGHTNESS 1

#endif

///@file led_sysdefs_arm_nrf52.h
#ifndef __LED_SYSDEFS_ARM_NRF52
#define __LED_SYSDEFS_ARM_NRF52

#define FASTLED_ARM

#ifndef F_CPU
    #define F_CPU 64000000 // the NRF52 series has a 64MHz CPU
#endif

// even though CPU is at 64MHz, use the 8MHz-defined timings because...
// PWM module   runs at 16MHz
// SPI0..2      runs at  8MHz
#define CLOCKLESS_FREQUENCY 16000000 // the NRF52 has EasyDMA for PWM module at 16MHz

#ifndef F_TIMER
    #define F_TIMER 16000000 // the NRF52 timer is 16MHz, even though CPU is 64MHz
#endif

#if !defined(FASTLED_USE_PROGMEM)
    #define FASTLED_USE_PROGMEM 0 // nRF52 series have flat memory model
#endif

#if !defined(FASTLED_ALLOW_INTERRUPTS)
    #define FASTLED_ALLOW_INTERRUPTS 1
#endif

// Use PWM instance 0
// See clockless_arm_nrf52.h and (in root of library) platforms.cpp
#define FASTLED_NRF52_ENABLE_PWM_INSTANCE0

#if defined(FASTLED_NRF52_NEVER_INLINE)
    #define FASTLED_NRF52_INLINE_ATTRIBUTE __attribute__((always_inline)) inline
#else     
    #define FASTLED_NRF52_INLINE_ATTRIBUTE __attribute__((always_inline)) inline
#endif    



#include <nrf.h>
#include <nrf_spim.h>   // for FastSPI
#include <nrf_pwm.h>    // for Clockless
#include <nrf_nvic.h>   // for Clockless / anything else using interrupts
typedef __I  uint32_t RoReg;
typedef __IO uint32_t RwReg;

#define cli()  __disable_irq()
#define sei()  __enable_irq()

#define FASTLED_NRF52_DEBUGPRINT(format, ...)\
//    do { FastLED_NRF52_DebugPrint(format, ##__VA_ARGS__); } while(0);

#endif // __LED_SYSDEFS_ARM_NRF52

#define CLKS_PER_US (F_CPU/1000000)