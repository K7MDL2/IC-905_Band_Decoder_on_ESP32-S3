/*
*   LED_Control.h
*
*   For the LED indicators on teh PCB (Band, PTT Out, PTT in, Power)
*   Jan 2025 K7MDL
*
*/

#include "IC905_ESP32-S3_PTT_Breakout.h"
#include "driver/ledc.h"

//#define RGB_LED  // use PWM mode to dim bright onboard LED. If undefined then gpio_set_level used, very bright though!

#ifdef RGB_LED

    #define LED_TIMER_BAND          LEDC_TIMER_0  // low duty cycle (dim) Band enabled
    #define LED_TIMER_FLASH         LEDC_TIMER_1  // Flash cadence used for when PTT is active.

    #define LEDC_MODE               LEDC_LOW_SPEED_MODE

    #define LEDC_PTT_IN_OUTPUT_IO      (48)     // Onboard RGB LED for testing
    //#define LEDC_PTT_IN_OUTPUT_IO      (35)     // PCB LED for PTT IN
    #define LEDC_OUTPUT_144_IO         (36)     // band 144 Active - Flash for PTT active on this band
    #define LEDC_OUTPUT_430_IO         (37)     // Skipping pin 38 as some boards use it for the internal RGB
    #define LEDC_OUTPUT_1200_IO        (39)
    #define LEDC_OUTPUT_2300_IO        (40)
    #define LEDC_OUTPUT_5600_IO        (41)
    #define LEDC_OUTPUT_10G_IO         (42)
    #define LEDC_OUTPUT_PWR_ON_IO      (47)

    #define LEDC_PTT_IN_OUTPUT_CH      (LEDC_CHANNEL_0)  // PTT Input
    #define LEDC_OUTPUT_144_CH         (LEDC_CHANNEL_1)  // band 144 Active - Flash for PTT active on this band
    #define LEDC_OUTPUT_430_CH         (LEDC_CHANNEL_2)
    #define LEDC_OUTPUT_1200_CH        (LEDC_CHANNEL_3)
    #define LEDC_OUTPUT_2300_CH        (LEDC_CHANNEL_4)
    #define LEDC_OUTPUT_5600_CH        (LEDC_CHANNEL_5)
    #define LEDC_OUTPUT_10G_CH         (LEDC_CHANNEL_6)
    #define LEDC_OUTPUT_PWR_ON_CH      (LEDC_CHANNEL_7)

    #define LEDC_DUTY_RES              LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
    #define LEDC_ON_DUTY               (LED_BRIGHT_LEVEL) // 50% is: (2 ** 13) * 50% = 4096.  Higher is brighter
    #define LEDC_OFF_DUTY              (0) // Set duty to 0%. (2 ** 13) * 50% = 4096.  Higher is brighter
    #define LEDC_FREQUENCY             (4000) // Frequency in Hertz. Set frequency at 4 kHz

    /* Warning:
    * For ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C2, ESP32C6, ESP32H2, ESP32P4 targets,
    * when LEDC_DUTY_RES selects the maximum duty resolution (i.e. value equal to SOC_LEDC_TIMER_BIT_WIDTH),
    * 100% duty cycle is not reachable (duty cannot be set to (2 ** SOC_LEDC_TIMER_BIT_WIDTH)).
    */

    void ledc_init(void);

#endif