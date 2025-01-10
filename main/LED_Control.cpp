/*
 *  LED_Control.cpp
 *  Jan 2025 by K7MDL
 *
 *   Operates the 8 PCB LEDs. 
 *   The onboard RGB LED is used for testing PTT in during dev.  Swap code to pin 35 when on PCB.
 *   The 8 LEDS are Power ON, PTT in, Band 1-6.  PTT for Bands 1-6 will flash the Band LED
 *   Some DevKitC boards use pin 38 for the onboard RGB so I left that pin unassigned.  Using pin 48 for testing.
 * 
*/ 

#include "freertos/FreeRTOS.h"  // need for time delay function
#include "LED_Control.h"

#ifdef RGB_LED
    void ledc_init(void)
    {
        // Prepare and then apply the LEDC PWM timer configuration
        ledc_timer_config_t ledc_timer[2] = {
            {
                .speed_mode       = LEDC_MODE,
                .duty_resolution  = LEDC_DUTY_RES,
                .timer_num        = LED_TIMER_BAND,
                .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
                .clk_cfg          = LEDC_AUTO_CLK,
                .deconfigure      = false
            },
            {
                .speed_mode       = LEDC_MODE,
                .duty_resolution  = LEDC_DUTY_RES,
                .timer_num        = LED_TIMER_FLASH,
                .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
                .clk_cfg          = LEDC_AUTO_CLK,
                .deconfigure      = false
            }
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer[LED_TIMER_BAND]));
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer[LED_TIMER_FLASH]));

        // Prepare and then apply the LEDC PWM channel configuration
        ledc_channel_config_t ledc_channel[8] = {
            {
                .gpio_num       = LEDC_PTT_IN_OUTPUT_IO,  // PTT-In LED output IO pin number
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_0,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_BAND,
                .duty           = 30, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            },
            {
                .gpio_num       = LEDC_OUTPUT_144_IO,
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_1,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_BAND,
                .duty           = 30, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            },
            {
                .gpio_num       = LEDC_OUTPUT_430_IO,
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_2,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_BAND,
                .duty           = 30, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            },
            {
                .gpio_num       = LEDC_OUTPUT_1200_IO,
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_3,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_BAND,
                .duty           = 30, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            },
            {
                .gpio_num       = LEDC_OUTPUT_2300_IO,
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_4,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_BAND,
                .duty           = 30, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            },
            {
                .gpio_num       = LEDC_OUTPUT_5600_IO,
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_5,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_BAND,
                .duty           = 30, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            },
            {
                .gpio_num       = LEDC_OUTPUT_10G_IO,
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_6,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_BAND,
                .duty           = 30, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            },
            {
                .gpio_num       = LEDC_OUTPUT_PWR_ON_IO,
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_7,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_BAND,
                .duty           = 30, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            }
        };
        
        for (int i=0; i< 8; i++) {
            if (i==0) {
                ledc_channel[i].flags.output_invert = 1;   // Enable (1) or disable (0) gpio output invert
                ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i]));
            } else{
                ledc_channel[i].flags.output_invert = 0;   // Enable (1) or disable (0) gpio output invert
                ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i]));    
            }
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, (ledc_channel_t) i, LEDC_ON_DUTY));//  Bands PTT, Bands 1-6 and PwrOn
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, (ledc_channel_t) i)); // Update duty to apply the new value 
        }
        ESP_LOGI("LED", "LED Bootup Lamp Test");
        vTaskDelay(pdMS_TO_TICKS(500));  // Short lamp test, with all on.
        for (int i=0; i< 8; i++) {
            //  Bands PTT, Bands 1-6 and PwrOn 
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, (ledc_channel_t) i, LEDC_OFF_DUTY));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, (ledc_channel_t) i)); // Update duty to apply the new value
        }
    }
#endif
