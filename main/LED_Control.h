/*
*   LED_Control.h
*
*   For the LED indicators on teh PCB (Band, PTT Out, PTT in, Power)
*   Jan 2025 K7MDL
*
*/

#include "IC905_ESP32-S3_PTT_Breakout.h"
#include "driver/ledc.h"

// use PWM mode to dim bright onboard LED. If USE_LEDS is undefined then gpio_set_level used, very bright though!

#ifdef USE_LEDS


    #define LED_TIMER_PTT               LEDC_TIMER_0  // Flash cadence used for when PTT is active.
    #define LED_TIMER_PWR_ON            LEDC_TIMER_1  // Flash cadence used for when PTT is active.
    #define LED_TIMER_BAND              LEDC_TIMER_2  // low duty cycle (dim) Band enabled
    
    #define LEDC_MODE                   LEDC_LOW_SPEED_MODE

    #ifdef PROTOTYPE
        #define LED_DIMMER_ADJ_PIN 18  // do not use 8 and 9, they are used for i2c in future. I
        // if using LEDs then OLED is not connected, these are free as gpio but even with a LCD a adj pot could still be useful

        // 45, 46, 0 and 3 are "strapping pins"  
        //#define LEDC_PTT_IN_OUTPUT_IO      (GPIO_NUM_48)     // Onboard RGB LED for testing
        #define LEDC_PTT_IN_OUTPUT_IO      (GPIO_NUM_47)     // PCB LED for PTT IN
        #define LEDC_OUTPUT_144_IO         (GPIO_NUM_35)     // band 144 Active - Flash for PTT active on this band
        #define LEDC_OUTPUT_430_IO         (GPIO_NUM_37)     // Skipping pin 38 as some boards use it for the internal RGB
        #define LEDC_OUTPUT_1200_IO        (GPIO_NUM_39)
        #define LEDC_OUTPUT_2300_IO        (GPIO_NUM_41)
        #define LEDC_OUTPUT_5600_IO        (GPIO_NUM_2)
        #define LEDC_OUTPUT_10G_IO         (GPIO_NUM_42)
        #define LEDC_OUTPUT_PWR_ON_IO      (GPIO_NUM_40)     // 43 and 44 are UART0 pins
    #else  // PCB V1
        #define LED_DIMMER_ADJ_PIN 4  // do not use 8 and 9, they are used for i2c in future. I
        // if using LEDs then OLED is not connected, these are free as gpio but even with a LCD a adj pot could still be useful

        // 45, 46, 0 and 3 are "strapping pins"  
        //#define LEDC_PTT_IN_OUTPUT_IO      (GPIO_NUM_48)     // Onboard RGB LED for testing
        #define LEDC_PTT_IN_OUTPUT_IO      (GPIO_NUM_35)     // PCB LED for PTT IN
        #define LEDC_OUTPUT_144_IO         (GPIO_NUM_36)     // band 144 Active - Flash for PTT active on this band
        #define LEDC_OUTPUT_430_IO         (GPIO_NUM_37)     // Skipping pin 38 as some boards use it for the internal RGB
        #define LEDC_OUTPUT_1200_IO        (GPIO_NUM_38)
        #define LEDC_OUTPUT_2300_IO        (GPIO_NUM_39)
        #define LEDC_OUTPUT_5600_IO        (GPIO_NUM_40)
        #define LEDC_OUTPUT_10G_IO         (GPIO_NUM_41)
        #define LEDC_OUTPUT_PWR_ON_IO      (GPIO_NUM_42)     // 43 and 44 are UART0 pins
    #endif

    #define LEDC_PTT_IN_OUTPUT_CH      (LEDC_CHANNEL_0)  // PTT Input
    #define LEDC_OUTPUT_144_CH         (LEDC_CHANNEL_1)  // band 144 Active - Flash for PTT active on this band
    #define LEDC_OUTPUT_430_CH         (LEDC_CHANNEL_2)
    #define LEDC_OUTPUT_1200_CH        (LEDC_CHANNEL_3)
    #define LEDC_OUTPUT_2300_CH        (LEDC_CHANNEL_4)
    #define LEDC_OUTPUT_5600_CH        (LEDC_CHANNEL_5)
    #define LEDC_OUTPUT_10G_CH         (LEDC_CHANNEL_6)
    #define LEDC_OUTPUT_PWR_ON_CH      (LEDC_CHANNEL_7)
    
    #define LEDC_DUTY_RES              LEDC_TIMER_13_BIT // Set duty resolution to 13 bits.  2 ** 13) = 8192.  Duty cycle is 2x duty res so 8192
    #define LEDC_ON_DUTY               (LED_BRIGHT_LEVEL) // for 13 bit 50% is: (2 ** 13) * 50% = 4096.  Higher is brighter
    #define GREEN_DIM_FACTOR           (3) // Green LED uses lower voltage so has higher current when using same size resistors as blue and red LEDs.
    #define LEDC_OFF_DUTY              (0) // Set duty to 0%. (2 ** 13) * 50% = 4096.  Higher is brighter
    #define LEDC_FREQUENCY             (4000) // Frequency in Hertz. Set frequency at 4 kHz

    /* Warning:
    * For ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C2, ESP32C6, ESP32H2, ESP32P4 targets,
    * when LEDC_DUTY_RES selects the maximum duty resolution (i.e. value equal to SOC_LEDC_TIMER_BIT_WIDTH),
    * 100% duty cycle is not reachable (duty cannot be set to (2 ** SOC_LEDC_TIMER_BIT_WIDTH)).
    */

    void ledc_init(void);
    void led_brightness(void);
    void PowerOn_LED(uint8_t state);
    void flash_PTT_LED(bool state, ledc_channel_t channel) ;

#endif