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
#include "esp_log.h"
#include "esp_timer.h"
#include "LED_Control.h"

extern uint32_t led_bright_level;
extern bool PTT;
extern uint8_t band;

#ifdef USE_LEDS
    void ledc_init(void)
    {
        // Prepare and then apply the LEDC PWM timer configuration
        ledc_timer_config_t ledc_timer[3] = {
            {
                .speed_mode       = LEDC_MODE,
                .duty_resolution  = LEDC_DUTY_RES,
                .timer_num        = LED_TIMER_PTT,
                .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
                .clk_cfg          = LEDC_AUTO_CLK,
                .deconfigure      = false
            },
            {
                .speed_mode       = LEDC_MODE,
                .duty_resolution  = LEDC_DUTY_RES,
                .timer_num        = LED_TIMER_PWR_ON,
                .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
                .clk_cfg          = LEDC_AUTO_CLK,
                .deconfigure      = false
            },
            {
                .speed_mode       = LEDC_MODE,
                .duty_resolution  = LEDC_DUTY_RES,
                .timer_num        = LED_TIMER_BAND,
                .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
                .clk_cfg          = LEDC_AUTO_CLK,
                .deconfigure      = false
            }
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer[LED_TIMER_PTT]));
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer[LED_TIMER_PWR_ON]));
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer[LED_TIMER_BAND]));

        // Prepare and then apply the LEDC PWM channel configuration
        ledc_channel_config_t ledc_channel[8] = {
            {
                .gpio_num       = LEDC_PTT_IN_OUTPUT_IO,  // PTT-In LED output IO pin number
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_0,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_PTT,
                .duty           = LEDC_ON_DUTY, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            },
            {
                .gpio_num       = LEDC_OUTPUT_144_IO,
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_1,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_BAND,
                .duty           = LEDC_ON_DUTY, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            },
            {
                .gpio_num       = LEDC_OUTPUT_430_IO,
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_2,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_BAND,
                .duty           = LEDC_ON_DUTY, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            },
            {
                .gpio_num       = LEDC_OUTPUT_1200_IO,
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_3,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_BAND,
                .duty           = LEDC_ON_DUTY, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            },
            {
                .gpio_num       = LEDC_OUTPUT_2300_IO,
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_4,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_BAND,
                .duty           = LEDC_ON_DUTY, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            },
            {
                .gpio_num       = LEDC_OUTPUT_5600_IO,
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_5,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_BAND,
                .duty           = LEDC_ON_DUTY, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            },
            {
                .gpio_num       = LEDC_OUTPUT_10G_IO,
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_6,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_BAND,
                .duty           = LEDC_ON_DUTY, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            },
            {
                .gpio_num       = LEDC_OUTPUT_PWR_ON_IO,
                .speed_mode     = LEDC_MODE,
                .channel        = LEDC_CHANNEL_7,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LED_TIMER_PWR_ON,
                .duty           = LEDC_ON_DUTY/GREEN_DIM_FACTOR, // Set duty to 0%
                .hpoint         = 0  // LEDC channel hpoint value, the range is [0, (2**duty_resolution)-1]
            }
        };

        // initialize and turn them all on for lamp test.
        for (int i=0; i< 8; i++) {
            //ledc_channel[i].flags.output_invert = 0;   // Enable (1) or disable (0) gpio output invert
            ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i]));
        }
        
        ESP_LOGI("LED", "LED Bootup Lamp Test");
        vTaskDelay(pdMS_TO_TICKS(1000));  // Short lamp test, with all on.

        for (int i=0; i< 8; i++) { // Turn them all off for end of lamp test
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, (ledc_channel_t) i, LEDC_OFF_DUTY));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, (ledc_channel_t) i)); // Update duty to apply the new value
        }
    }

    // Adjust all the LED's duty cycle settings to control brightness.  Usually using value from ADC.
    void led_brightness(void) {  //  Bands PTT, Bands 1-6 and PwrOn 
        if (band == 0) return;   // do nto have a valid band from radio yet, wait.
        
        // All LEDS are turning on and off by their duty cycle settings.
        if (ledc_get_duty(LEDC_MODE, LEDC_OUTPUT_PWR_ON_CH) > 0)
        {
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_PWR_ON_CH, led_bright_level/GREEN_DIM_FACTOR));  // Dim the green LED
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_OUTPUT_PWR_ON_CH)); // Keep the power on led lit
        }
        
        for (int i=0; i< 7; i++) {  // Adjust the bands level) .  
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, (ledc_channel_t) i, led_bright_level));
            if (PTT && i==0) {
               ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_PTT_IN_OUTPUT_CH));
            }
            if (i > 0 && i == band) {
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, (ledc_channel_t) i)); // only turn on the active Band LED
            }
        }
    }

    void PowerOn_LED(uint8_t state)
    {   
        static uint64_t flash_timer = esp_timer_get_time();
        uint64_t flash_time = 500;  // in ms
        bool update_LED = false;

        switch (state) {
            case 2:    // Flash the power light when we lose USB connection to the radio
            {
                // Flash power light on 
                if (esp_timer_get_time() >= flash_timer + flash_time *1000) {  
                    if (ledc_get_duty(LEDC_MODE, LEDC_OUTPUT_PWR_ON_CH) > 0) {
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_PWR_ON_CH, LEDC_OFF_DUTY));    
                        //ESP_LOGI("PowerOn_LED", "Flash LED OFF");
                    } else {
                       ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_PWR_ON_CH, led_bright_level/GREEN_DIM_FACTOR));  // Dim the green LED
                       //ESP_LOGI("PowerOn_LED", "Flash LED ON");
                    }
                    flash_timer = esp_timer_get_time();
                    update_LED = true;
                }
                break;
            }

            case 1: // Turn on power light when we have a valid USB connection to the radio
                    if (ledc_get_duty(LEDC_MODE, LEDC_OUTPUT_PWR_ON_CH) == 0) //  do not turn on if it is already on
                    {
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_PWR_ON_CH, led_bright_level/GREEN_DIM_FACTOR));  // Dim the green LED
                        update_LED = true;
                    }
                    break;
            
            case 0: // Turn off power LED  
            default: 
                    if (ledc_get_duty(LEDC_MODE, LEDC_OUTPUT_PWR_ON_CH) > 0) //  do not turn oFF if it is already off
                    {
                        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_PWR_ON_CH, LEDC_OFF_DUTY));
                        update_LED = true;
                    }
        }
        if (update_LED) {
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_OUTPUT_PWR_ON_CH)); // Update duty to apply the new value
            update_LED = false;
        }
    }
#endif

void flash_PTT_LED(bool state, ledc_channel_t channel) 
{
    static uint64_t flash_timer = 0; // = esp_timer_get_time();
    uint64_t flash_time = 300;  // 300 ms.
    bool update_LED = false;
  
    if (state == 1)  // flash LED channel
    {
        if (esp_timer_get_time() >= flash_timer + flash_time *1000) {  
            if (ledc_get_duty(LEDC_MODE, channel) > 0) {
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, LEDC_OFF_DUTY));    
                //ESP_LOGI("flash_PTT_LED", "Flash LED OFF");
            } else {
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, led_bright_level));  // Turn on and Dim the LED
                //ESP_LOGI("flash_PTT_LED", "Flash LED ON");
            }        
            flash_timer = esp_timer_get_time();
            update_LED = true;
        }
    }
    
    if (state == 0)  // Turn LED channel on Solid
    {
        //ESP_LOGI("flash_PTT_LED", "Read LED state = %lu", ledc_get_duty(LEDC_MODE, channel));
        if (ledc_get_duty(LEDC_MODE, channel) == 0) {
            //ESP_LOGI("flash_PTT_LED", "End Flashing, Turn band LED back to solid ON state");
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, led_bright_level));  // Turn on and Dim the LED
            update_LED = true;
        }
    }
    
    if (update_LED)   // update to take effect
    {
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel)); // Update duty to apply the new value
        update_LED = false;
    }
}