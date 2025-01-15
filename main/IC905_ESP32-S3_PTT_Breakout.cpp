/*
 *   IC905_ESP32-S3_PTT_Breakout.cpp
 *   Jan 2025 by K7MDL
 *
 *  This program is a USB Host Serial device that reads the Icom IC-905 USB CI_V frequency messages.
 *  For 10GHz and higher the IC-905 adds 1 extra byte that must be accounted for to decode the BCD encoded frequency
 *  The last data byte sent for frequency is the MSB.
 *  The radio PTT (aka SEND output) is wired into this CPU and monitored.  There is a status LED for it.
 *  There are 6 PTT and 6 band enable outputs, with status LEDs for each band
 *  There is a power LED.  It is assigned for power status but can be used for anything.
 *  PTT is normally +5V if directly connected to the SEND jack with a weak pullup at the radio and at the CPU sides
 *  When PTT = 0 (radio is in TX) then we use the last frequency seen and send logic 1 to the matching band's PTT output.
 *  A ULN2803A open collector octal driver buffers the CPU outputs and also inverts so a logic 1 results in closure to GND.
 * 
 *  Support for the AtomS3 is enabled by a #define ATOMS3. It has no I/O support in this code today but has a nice
 *  small color LCD that displays frequency, time/date from radio, BAND, TX status, and location.
 *  I caclulate 8 digit grid square and display it in teh debug and on an optional display like the AtomsS3
 *  On bootup the radio is polled for time, location, band/frequency, modem, extended mode, UTC offset.  
 *  After that it listens for messages only so it does not interfere with future PC to Radio CI_V serial bridging.
 * 
 *  ---------------------------------------------------------------------------------
 * The USB Serial Host part of this program is based on cdc_acm_host example file from 
 * 
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "IC905_ESP32-S3_PTT_Breakout.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "esp_timer.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "time.h"
#include <sys/time.h>
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "Decoder.h"
#include "CIV.h"
#include "LED_Control.h"

//#define USB_PC

#ifdef USB_PC
    #include "tusb_cdc_acm.h"
    #include "tinyusb.h"
    #include "sdkconfig.h"
    //#define CONFIG_TINYUSB_CDC_RX_BUFSIZE 512
    static uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
#endif

// Our own functions
void display_Freq(uint64_t _freq, bool _force);
void display_Time(uint8_t _UTC, bool _force);
void display_PTT(bool _PTT_state, bool _force);
void display_Band(uint8_t _band, bool _force);
void display_Grid(char _grid[], bool _force);
void SetFreq(uint64_t Freq);
void poll_for_time(void);

struct Bands bands[NUM_OF_BANDS] = {
  { "DUMMY", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF },                                       // DUMMY Band to avoid using 0
  { "2M", 144000000, 148000000, 0, 144200000, 1, 1, 0, 1, 0, 0, 0, DECODE_INPUT_BAND144 },                  // 2m
  { "70cm", 430000000, 450000000, 0, 432100000, 1, 1, 0, 1, 0, 0, 0, DECODE_INPUT_BAND430 },                // 430/440  No LO
  { "23cm", 1240000000, 1300000000, 0, 1296100000, 1, 1, 0, 1, 0, 0, 0, DECODE_INPUT_BAND1200 },            // 1296Mhz  with 144Mhz LO
  { "13cm", 2300000000, 2450000000, 0, 2304100000, 1, 1, 0, 1, 0, 0, 0, DECODE_INPUT_BAND2300 },            // 2.3 and 2.4GHz
  { "6cm",  5650000000, 5925000000, 0, 5760100000, 1, 1, 0, 1, 0, 0, 0, DECODE_INPUT_BAND5600 },            // 5.7GHz
  { "3cm", 10000000000, 10500000000, 0, 10368100000, 1, 1, 0, 1, 0, 0, 0, DECODE_INPUT_BAND10G },           // 10GHz
};

char title[25] = "IC-905 Band Decoder";
uint16_t baud_rate;                   //Current baud speed
uint32_t readtimeout = 10;            //Serial port read timeout
uint8_t band = BAND_2M;
#ifdef ATOMS3
    uint16_t background_color = TFT_BLACK;
#endif
bool PTT = false;
bool prev_PTT = true;
extern char Grid_Square[];
extern struct Modes_List modeList[];
//extern char FilStr;
extern struct cmdList cmd_List[];
uint8_t radio_address = RADIO_ADDR;  //Transceiver address.  0 allows auto-detect on first messages form radio
bool use_wired_PTT = WIRED_PTT;           // Selects source of PTT, wired input or polled state from radio.  Wired is preferred, faster.
uint8_t read_buffer[64];  //Read buffer
uint8_t prev_band = 0xFF;
uint64_t prev_frequency = 0;
uint16_t poll_radio_ptt = POLL_PTT_DEFAULT;  // can be changed with detected radio address.
uint8_t UTC = 1;  // 0 local time, 1 UTC time
bool update_radio_settings_flag = false;
extern bool BtnA_pressed;
extern bool BtnB_pressed;
extern bool BtnC_pressed;
uint8_t readLine(void);
void processCatMessages(const uint8_t *pData, size_t data_len);
void sendCatRequest(const uint8_t cmd_num, const uint8_t Data[], const uint8_t Data_len);  // first byte in Data is length
void printFrequency(void);
void read_Frequency(uint64_t freq, uint8_t data_len);
uint8_t getBand(uint64_t _freq);
void refresh_display(void);
void draw_new_screen(void);
uint8_t formatFreq(uint64_t vfo, uint8_t vfo_dec[]);
extern void CIV_Action(const uint8_t cmd_num, const uint8_t data_start_idx, const uint8_t data_len, const uint8_t msg_len, const uint8_t *rd_buffer);
uint8_t pass_PC_to_radio(void);
extern uint8_t USBHost_ready;  // 0 = not mounted.  1 = mounted, 2 = system not initialized
bool USBH_connected = false;
extern uint16_t background_color;
uint64_t frequency = 0;
extern bool update_radio_settings_flag;
static const char *TAG = "USB-CDC";
static SemaphoreHandle_t device_disconnected_sem;
uint8_t *r = read_buffer;
bool msg_done_flag;
bool get_ext_mode_flag = false;
cdc_acm_dev_hdl_t cdc_dev = NULL;
uint8_t board_type = 0;
bool BLE_connected = 0;
bool btConnected = 0;
uint8_t brightness = 110;   // 0-255
uint8_t rotation   = 3;   // 0-3
struct tm tm;
time_t t;
bool init_done = 0;
bool sending = 0;
static int adc_raw[1][10];
static int voltage[1][10];
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);
uint8_t radio_address_received = 0;

// ADC setup
adc_cali_handle_t adc2_cali_chan0_handle = NULL;
bool do_calibration2 = adc_calibration_init(ADC_UNIT_2, ADC2_CHAN0, ADC_ATTEN, &adc2_cali_chan0_handle);

adc_oneshot_unit_handle_t adc2_handle;
adc_oneshot_unit_init_cfg_t init_config2 = {
    .unit_id = ADC_UNIT_2,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
};

uint32_t led_bright_level = LED_BRIGHT_LEVEL;   // New level to set LEDs to

// Mask all 12 of our Band and PTT output pins for Output mode, also our 8 LED output pins
#ifdef USE_LEDS                            
    #define GPIO_OUTPUT_PIN_SEL ( 1ULL<<GPIO_BAND_OUTPUT_144  | 1ULL<<GPIO_BAND_OUTPUT_430  | 1ULL<<GPIO_BAND_OUTPUT_1200 \
                                | 1ULL<<GPIO_BAND_OUTPUT_2300 | 1ULL<<GPIO_BAND_OUTPUT_5600 | 1ULL<<GPIO_BAND_OUTPUT_10G  \
                                | 1ULL<<GPIO_PTT_OUTPUT_144   | 1ULL<<GPIO_PTT_OUTPUT_430   | 1ULL<<GPIO_PTT_OUTPUT_1200  \
                                | 1ULL<<GPIO_PTT_OUTPUT_2300  | 1ULL<<GPIO_PTT_OUTPUT_5600  | 1ULL<<GPIO_PTT_OUTPUT_10G   \
                                | 1ULL<<LEDC_OUTPUT_144_IO    | 1ULL<<LEDC_OUTPUT_430_IO    | 1ULL<<LEDC_OUTPUT_1200_IO   \
                                | 1ULL<<LEDC_OUTPUT_2300_IO   | 1ULL<<LEDC_OUTPUT_5600_IO   | 1ULL<<LEDC_OUTPUT_10G_IO    \
                                | 1ULL<<LEDC_OUTPUT_PWR_ON_IO | 1ULL<<LEDC_PTT_IN_OUTPUT_IO );
#else
    #define GPIO_OUTPUT_PIN_SEL ( 1ULL<<GPIO_BAND_OUTPUT_144  | 1ULL<<GPIO_BAND_OUTPUT_430  | 1ULL<<GPIO_BAND_OUTPUT_1200 \
                                | 1ULL<<GPIO_BAND_OUTPUT_2300 | 1ULL<<GPIO_BAND_OUTPUT_5600 | 1ULL<<GPIO_BAND_OUTPUT_10G  \
                                | 1ULL<<GPIO_PTT_OUTPUT_144   | 1ULL<<GPIO_PTT_OUTPUT_430   | 1ULL<<GPIO_PTT_OUTPUT_1200  \
                                | 1ULL<<GPIO_PTT_OUTPUT_2300  | 1ULL<<GPIO_PTT_OUTPUT_5600  | 1ULL<<GPIO_PTT_OUTPUT_10G   );
#endif                            

// Mask our 1 PTT input IO pin for Input with pull up. An interrupt wil be assigned to this pin.
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_PTT_INPUT)
#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;

/**
 * Brief:
 * This GPIO code configures gpio outputs fro band and PTT and the gpio input interrupt on the PTT input
 *
 * GPIO status:
 * GPIO5 : output
 * GPIO6 : output
 * GPIO7 : output
 * GPIO10 : output
 * GPIO11 : output
 * GPIO12 : output
 * GPIO1:  input, pulled up, interrupt from rising edge and falling edge
 *
 * add 6 More for PTT pins
 * 
 * Test:
 * Monitor voltage at pins
 *
 */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// display PTT Input status to screen
// Set PTT Flag and service it in the main loop. 
static void gpio_PTT_Input(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            PTT = (uint8_t) gpio_get_level((gpio_num_t) io_num);  // Invert for buffer
            ESP_LOGI(TAG,"GPIO[%lu] intr, val: %d", io_num, PTT);
            PTT_Output(band, PTT);
            display_PTT(PTT, false);
            
            #ifdef USE_LEDS  // toggle PTT LED
                if (PTT) {
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_PTT_IN_OUTPUT_CH, led_bright_level));
                    //ESP_ERROR_CHECK(ledc_timer_resume(LEDC_MODE, LED_TIMER_PTT));
                } else {
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_PTT_IN_OUTPUT_CH, LEDC_OFF_DUTY));
                    //ESP_ERROR_CHECK(ledc_timer_pause(LEDC_MODE, LED_TIMER_PTT));
                }
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_PTT_IN_OUTPUT_CH));
            #else
                // 100% ON or OFF.  This is too bright so use PWM method preferred
                gpio_set_level(GPIO_NUM_48, !PTT);   // Pin is Open Drain, set to 0 to close to GND and turn on LED.
                refresh_display();
            #endif
        }
    }
}

#ifdef USB_PC
    /**
     * @brief Application Queue
     */
    static QueueHandle_t app_queue;
    typedef struct {
        uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];     // Data buffer
        size_t buf_len;                                     // Number of bytes received
        uint8_t itf;                                        // Index of CDC device interface
    } app_message_t;

    app_message_t msg;

    /**
     * @brief CDC device RX callback
     *
     * CDC device signals, that new data were received
     *
     * @param[in] itf   CDC device index
     * @param[in] event CDC event type
     */
    void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
    {
        /* initialization */
        size_t rx_size = 0;

        /* read */
        esp_err_t ret = tinyusb_cdcacm_read(itf, rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
        if (ret == ESP_OK) {

            app_message_t tx_msg = {
                .buf_len = rx_size,
                .itf = itf,
            };

            memcpy(tx_msg.buf, rx_buf, rx_size);
            xQueueSend(app_queue, &tx_msg, 0);
        } else {
            ESP_LOGE(TAG, "Read Error");
        }
    }

    /**
     * @brief CDC device line change callback
     *
     * CDC device signals, that the DTR, RTS states changed
     *
     * @param[in] itf   CDC device index
     * @param[in] event CDC event type
     */
    void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
    {
        int dtr = event->line_state_changed_data.dtr;
        int rts = event->line_state_changed_data.rts;
        ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
    }
#endif // USB_PC


/**
 * @brief Data received callback
 *
 * @param[in] pData     Pointer to received data
 * @param[in] data_len Length of received data in bytes
 * @param[in] arg      Argument we passed to the device open function
 * @return
 *   true:  We have processed the received data
 *   false: We expect more data
 */
static bool handle_rx(const uint8_t *pData, size_t data_len, void *arg)
{
    ESP_LOG_BUFFER_HEXDUMP("handle_rx", pData, data_len, ESP_LOG_INFO);

    if (pData[data_len-1] == 0xFD)
    {
        //ESP_LOG_BUFFER_HEXDUMP("handle_rx-1", pData, data_len, ESP_LOG_INFO);
        
        #ifdef PC_PASSTHROUGH
            /* bridge Radio --> PC */
            tinyusb_cdcacm_write_queue(msg.itf, pData, data_len);
            esp_err_t err = tinyusb_cdcacm_write_flush(msg.itf, 0);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "CDC ACM write flush error: %s", esp_err_to_name(err));
            }
        #endif
        processCatMessages(pData, data_len);
    }
    return true;
}

/**
 * @brief Device event callback
 *
 * Apart from handling device disconnection it doesn't do anything useful
 *
 * @param[in] event    Device event type and data
 * @param[in] user_ctx Argument we passed to the device open function
 */
static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM error has occurred, err_no = %i", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "Device suddenly disconnected");
        ESP_ERROR_CHECK(cdc_acm_host_close(event->data.cdc_hdl));
        xSemaphoreGive(device_disconnected_sem);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGI(TAG, "Serial state notif 0x%04X", event->data.serial_state.val);
        break;
    case CDC_ACM_HOST_NETWORK_CONNECTION:
    default:
        ESP_LOGW(TAG, "Unsupported CDC event: %i", event->type);
        break;
    }
}

/**
 * @brief USB Host library handling task
 *
 * @param arg Unused
 */
static void usb_lib_task(void *arg)
{
    while (1) {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
            // Continue handling USB events to allow device reconnection
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief USB Host Radio TX message send and idle loop task
 *
 * @param arg Unused
 */
static void usb_loop_task(void *arg)
{
    static uint32_t last_level = 0;  // remember the last LED brightness level
    while (1) {  
        if (get_ext_mode_flag) {
            ESP_LOGI(TAG, "***Get extended mode info from radio");
            sendCatRequest(CIV_C_F26A, 0, 0);  // Get extended info -  mode, filter, and datamode status
            get_ext_mode_flag = false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
       
        #ifndef USE_LEDS
            refresh_display();
        #else 
            vTaskDelay(pdMS_TO_TICKS(10));

            // Read ADC1 Ch7 (pin 8) for LED brightness POT position.
            ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC2_CHAN0, &adc_raw[0][0]));
            //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_2 + 1, ADC2_CHAN0, adc_raw[0][0]);
            if (do_calibration2) {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
                //ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_2 + 1, ADC2_CHAN0, voltage[0][0]);
            }

            // invert the ADC because the pot power and ground leads are reversed due board edge ground trace being nearby. Can change in PCB
            led_bright_level= (8192 - (adc_raw[0][0] *2)) ;  // 12 bits ADC output range. 
            //  Multiply by 2 to get 8192 for Ledc duty res of 13 bits. Keep < 8190.
            
            vTaskDelay(pdMS_TO_TICKS(100));
            
            if (init_done) {   // only spend CPU cycles changing the brightness level if it changes more than a small amount
                if ((led_bright_level < (last_level - 40)) || (led_bright_level > (last_level + 40))) {
                    if (led_bright_level < 1)
                        led_bright_level = 0;  // API will crash if exceed max resolution 
                    if (led_bright_level > 8100)
                        led_bright_level = 8100;  // API will crash if exceed max resolution
                    led_brightness();  // we now have a valid band so the leds get set correctly
                    ESP_LOGI(TAG, "Updated LED band brightness level to %lu of 0-8100", led_bright_level);
                    last_level = led_bright_level;
                }
            }
        #endif // USE_LEDS

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        if (!calibrated) {
            ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
            adc_cali_curve_fitting_config_t cali_config = {
                .unit_id = unit,
                .chan = channel,
                .atten = atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
            if (ret == ESP_OK) {
                calibrated = true;
            }
        }
    #endif

    #if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        if (!calibrated) {
            ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
            adc_cali_line_fitting_config_t cali_config = {
                .unit_id = unit,
                .atten = atten,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
            if (ret == ESP_OK) {
                calibrated = true;
            }
        }
    #endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)
{
    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
        ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

    #elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
        ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
    #endif
}

void printBinaryWithPadding(uint8_t num, char bin_num[9]) { 
    for (int i = sizeof(uint8_t) * 8 - 1; i >= 0; i--) {
        char b = ((num >> i) & 1) + 0x30;
        //printf("%c", b);
        bin_num[7-i] = b;
        //if (i % 4 == 0) printf(" "); // Group by 4 bits for readability
    }
    bin_num[8] = '\0';
    //ESP_LOGI(TAG,"binary is %s", bin_num);
}

// ----------------------------------------
//      Send CAT Request
// ----------------------------------------
void sendCatRequest(const uint8_t cmd_num, const uint8_t Data[], const uint8_t Data_len)  // first byte in Data is length
{
    int8_t msg_len;
    uint8_t req[50] = { START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS };

    ESP_LOGI("sendCatRequest", "USBH_connected = %d", USBH_connected);

    for (msg_len = 0; msg_len <= cmd_List[cmd_num].cmdData[0]; msg_len++)  // copy in 1 or more command bytes
        req[msg_len + 4] = cmd_List[cmd_num].cmdData[msg_len + 1];

    msg_len += 3;  // Tee up to add data if any

    uint8_t j = 0;
    if (Data_len != 0) {              // copy in 1 or more data bytes, if any
        for (j = 0; j < Data_len; j++)  //pick up with value i
        req[msg_len++] = Data[j];
    }

    req[msg_len] = STOP_BYTE;
    req[msg_len + 1] = 0;  // null terminate for printing or conversion to String type

    #ifdef RAW_TX
        for (uint8_t k = 0; k <= msg_len; k++) {
            ESP_LOGI("sendCatRequest --> Tx Raw Msg: ", "%X,", req[k]);
        }
        ESP_LOGI("sendCatRequest", " msg_len = %d   END", msg_len);
    #endif

    if (USBH_connected) {
        if (msg_len < sizeof(req) - 1) {  // ensure our data is not longer than our buffer
            //ESP_LOGI("sendCatRequest", "***Send CI-V Msg: ");
            int loop_ct = 0;
            while (sending && loop_ct < 5) {   // In case an overlapping send from IRQ call coes in, wait here until the first one is done.
                vTaskDelay(pdMS_TO_TICKS(10));    
                loop_ct++;
            }
            loop_ct = 0;
            sending = 1;
            ESP_LOGI("Send_CatRequest", "*** Send Cat Request Msg - result = %d  END TX MSG, msg_len = %d", cdc_acm_host_data_tx_blocking(cdc_dev, (const uint8_t *)req, msg_len+1, 1000), msg_len);
            vTaskDelay(pdMS_TO_TICKS(10));
            sending = 0;
        } 
        else 
        {
            ESP_LOGI("sendCatRequest", "Buffer overflow");
        }  // if overflow
    }    // if connected
}

// ----------------------------------------
//      Print the received frequency
//
//  This function is only called when the radio's reports a new frequency so is always the radio's real
//    frequency, which when in Xvtr mode, the CI-V message value is the IF frequency.
//
//  The value 'frequency' is a global and is the final displayed frequency, Xvtr band or not.
//    - On entry to this function, frequency is the global displayed value, may not be the actual radio (IF) frequency
//      which is yet to be extracted from the CI-V messaage.
//    - On exit from this function, frequency is either updated (non-XVtr bands) or Xvtr_offset applied, if any.
//
// ----------------------------------------
void read_Frequency(uint64_t freq, uint8_t data_len) {  // This is the displayed frequency, before the radio input, which may have offset applied
    if (frequency > 0) {                   // store frequency per band before it maybe changes bands.  Required to change IF and restore direct after use as an IF.
        //ESP_LOGI(TAG,"read_Frequency: Last Freq %-13llu", frequency);
        if (!update_radio_settings_flag) {   // wait until any XVTR transition complete
        bands[band].VFO_last = frequency;  // store Xvtr or non-Xvtr band displayed frequency per band before it changes.
        prev_band = band;                  // store associated band index
        }
    }  // if an Xvtr band, subtract the offset to get radio (IF) frequency

    // Could do more validation here. Freq Calculation moved to CIV.cpp
    frequency = freq;
    
    band = getBand(frequency);
    if (band != prev_band) {
        //ESP_LOGI("read_Frequency", "***Get extended mode data from radio after band change - Setting flag");
        //get_ext_mode_flag = true;
        
        #ifdef USE_LEDS
            if (prev_band != 0xFF) {  // have not been through here before, make sure we have a valid band to set the LED to.
                ESP_LOGI("read_Frequency", "*** Band change to %d.  prev_band was %d", band, prev_band);
                // Turn off LED for previous band
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, (ledc_channel_t) prev_band, LEDC_OFF_DUTY));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, (ledc_channel_t) prev_band));

                // light up LED for new band
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, (ledc_channel_t) band, led_bright_level));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, (ledc_channel_t) band));
            }
        #else
            draw_new_screen();
        #endif
        
        // Use the band to operate our band enable outputs for the 6 905 bands.
        Band_Decode_Output(band);

        prev_band = band;
    }

    ESP_LOGI(TAG,"read_Frequency: Freq %-13llu  band =  %d  datalen = %d   btConnected %d   USBH_connected %d  BLE_connected %d  radio_address %X", frequency, band, data_len, btConnected, USBH_connected, BLE_connected, radio_address);
    // On exit from this function we have a new displayed frequency that has XVTR_Offset added, if any.
}

// ----------------------------------------
//    get band from frequency
// ----------------------------------------
uint8_t getBand(uint64_t _freq) {
  for (uint8_t i = 0; i < NUM_OF_BANDS; i++) {
    if (_freq >= bands[i].edge_lower && _freq <= bands[i].edge_upper) {
      if (i >= NUM_OF_BANDS-1) return NUM_OF_BANDS-1;
      return i;
    }
  }
  return 0x00;  // no band for considered frequency found
}

// Send new frequency to radio, radio will change bands as needed.
// ToDo:  Radio mode and other settings are not touched so stay the same as the last band used.  We are only changing the frequency, nothing else.
//        Need to save mode, filter and other stuff to return each band to the last way it was used.
void SetFreq(uint64_t Freq) {
  uint8_t vfo_dec[7] = {};
  uint8_t len = formatFreq(Freq, vfo_dec);  // Convert to BCD string
  ESP_LOGI("SetFreq","Set Radio Freq to %llu  Bytes sent to radio (5 or 6 bytes) in BCD: %02X %02X %02X %02X %02X (%02X)", Freq, vfo_dec[0], vfo_dec[1], vfo_dec[2], vfo_dec[3], vfo_dec[4], vfo_dec[5]);
  sendCatRequest(CIV_C_F1_SEND, vfo_dec, len);
}

// Length is 5 or 6 depending if < 10GHz band  followed by 5 or 6 BCD encoded frequency bytes
// vfo_dec[] holds the frequency result to send out
// returns length;
uint8_t formatFreq(uint64_t vfo, uint8_t vfo_dec[]) {
  //static uint8_t vfo_dec[7] = {};  // hold 6 or 7 bytes (length + 5 or 6 for frequency, bcd encoded bytes)
  uint8_t len;

  if (vfo < 10000000000LL) {
    len = 5;
    vfo_dec[6] = (uint8_t)0x00;  // set to 0, unused < 10Ghz
    for (uint8_t i = 0; i < len; i++) {
      uint64_t x = vfo % 100;
      vfo_dec[i] = bcdByteEncode((uint8_t)(x));
      vfo = vfo / 100;
    }
    //ESP_LOGI(TAG," VFO: < 10G Bands = Reversed hex to DEC byte %02X %02X %02X %02X %02X %02X", vfo_dec[0], vfo_dec[1], vfo_dec[2], vfo_dec[3], vfo_dec[4], vfo_dec[5]);
  } else {
    len = 6;
    for (uint8_t i = 0; i < len; i++) {
      uint64_t x = vfo % 100;
      vfo_dec[i] = bcdByteEncode((uint8_t)(x));
      vfo = vfo / 100;
    }
    //ESP_LOGI(TAG," VFO: > 10G Bands = Reversed hex to DEC byte %02X %02X %02Xpass_PC_to_radio %02X %02X %02X %02X", vfo_dec[0], vfo_dec[1], vfo_dec[2], vfo_dec[3], vfo_dec[4], vfo_dec[5], vfo_dec[6]);
  }
  return len;  // 5 or 6
}

uint8_t IC_905_Input_scan(void) 
{
    uint8_t pattern = 0;
        pattern |= GPIO_PTT_INPUT;       // PTT input from Radio SEND jack  Gnd - TX
    return pattern & 0x01;  // single GPIO pins spare for now
}

// convert band number to a unique band output pattern
void Band_Decode_Output(uint8_t band) // pass IF_Switch_OFF value to GPIO_out for sequencing usage
{
    ESP_LOGI("  Band_Decode_Output", "band %s", bands[band].band_name);
    switch (band)  // Now set them to the desired ON state
    {   
        case  BAND_2M   : GPIO_Out(DECODE_BAND144);  break;   //2M
        case  BAND_70cm : GPIO_Out(DECODE_BAND430);  break;   //432
        case  BAND_23cm : GPIO_Out(DECODE_BAND1200); break;   //1296
        case  BAND_13cm : GPIO_Out(DECODE_BAND2300); break;   //2400
        case  BAND_6cm  : GPIO_Out(DECODE_BAND5600); break;   //5760M
        case  BAND_3cm  : GPIO_Out(DECODE_BAND10G);  break;   //10G
        default: GPIO_Out(DECODE_BAND144);
    }
}

void GPIO_Out(uint8_t pattern)
{
    char bin_num[9] ={};
    printBinaryWithPadding(pattern, bin_num);
    ESP_LOGI("  Band_Decode_Output", "pattern:  DEC=%d    HEX=%X   Binary=%s", pattern, pattern, bin_num);
    // write to the assigned pin for each band enable output port
    gpio_set_level(GPIO_BAND_OUTPUT_144,  (pattern & 0x01) ? 1 : 0);  // bit 0
    gpio_set_level(GPIO_BAND_OUTPUT_430,  (pattern & 0x02) ? 1 : 0);  // bit 1
    gpio_set_level(GPIO_BAND_OUTPUT_1200, (pattern & 0x04) ? 1 : 0);  // bit 2
    gpio_set_level(GPIO_BAND_OUTPUT_2300, (pattern & 0x08) ? 1 : 0);  // bit 3
    gpio_set_level(GPIO_BAND_OUTPUT_5600, (pattern & 0x10) ? 1 : 0);  // bit 4
    gpio_set_level(GPIO_BAND_OUTPUT_10G,  (pattern & 0x20) ? 1 : 0);  // bit 5
}

void PTT_Output(uint8_t band, bool PTT_state)
{
    ESP_LOGI("PTT_Output:", " Band %s   PTT_state %d", bands[band].band_name, PTT_state);
    switch (band)
    {
        case  DUMMY     : GPIO_PTT_Out(DECODE_DUMMY_PTT,    false);     break;   //Dummy Band
        case  BAND_2M   : GPIO_PTT_Out(DECODE_BAND144_PTT,  PTT_state); break;   //2M
        case  BAND_70cm : GPIO_PTT_Out(DECODE_BAND430_PTT,  PTT_state); break;   //432
        case  BAND_23cm : GPIO_PTT_Out(DECODE_BAND1200_PTT, PTT_state); break;   //1296
        case  BAND_13cm : GPIO_PTT_Out(DECODE_BAND2300_PTT, PTT_state); break;   //2400
        case  BAND_6cm  : GPIO_PTT_Out(DECODE_BAND5600_PTT, PTT_state); break;   //5760M
        case  BAND_3cm  : GPIO_PTT_Out(DECODE_BAND10G_PTT,  PTT_state); break;   //10.368.1G
        default: GPIO_PTT_Out(DECODE_BAND144_PTT,  PTT_state);
    }
}

void GPIO_PTT_Out(uint8_t pattern, bool _PTT_state)
{   
    uint8_t PTT_state = _PTT_state ? 0xFF : 0;

    char bin_num[9] ={};
    printBinaryWithPadding(pattern, bin_num);
    ESP_LOGI("GPIO_PTT_Out", "PTT Output Binary %s   PTT Output Hex 0x%02X   PTT Level at CPU pin %d", bin_num, pattern, _PTT_state);
    gpio_set_level(GPIO_PTT_OUTPUT_144,  (pattern & 0x01 & PTT_state) ? 1 : 0);  // bit 0 144
    gpio_set_level(GPIO_PTT_OUTPUT_430,  (pattern & 0x02 & PTT_state) ? 1 : 0);  // bit 1 430
    gpio_set_level(GPIO_PTT_OUTPUT_1200, (pattern & 0x04 & PTT_state) ? 1 : 0);  // bit 2 1200
    gpio_set_level(GPIO_PTT_OUTPUT_2300, (pattern & 0x08 & PTT_state) ? 1 : 0);  // bit 3 2300
    gpio_set_level(GPIO_PTT_OUTPUT_5600, (pattern & 0x10 & PTT_state) ? 1 : 0);  // bit 4 5600
    gpio_set_level(GPIO_PTT_OUTPUT_10G,  (pattern & 0x20 & PTT_state) ? 1 : 0);  // bit 5 10G
}

// --------------------------------------------------
//   Process the received messages from transceiver
// --------------------------------------------------
void processCatMessages(const uint8_t read_buffer[], size_t data_len) {
    /*
        <FE FE E0 42 04 00 01 FD  - LSB
        <FE FE E0 42 03 00 00 58 45 01 FD  -145.580.000

        FE FE - start bytes
        00/E0 - target address (broadcast/controller)
        42 - source address
        00/03 - data type
        <data>
        FD - stop byte
    */

    uint8_t cmd_num;
    uint8_t match = 0;
    uint8_t data_start_idx = 0;

    if (1) {
        //bool knowncommand = true;
        int i;
        uint8_t msg_len = 0;

        cmd_num = 255;
            match = 0;

        if ((msg_len = data_len) > 0) {
            //#define SEE_RAW_RX
            #ifdef SEE_RAW_RX
                ESP_LOGI("processCatMessages", "<++ Rx Raw Msg: ");
                for (uint8_t k = 0; k < msg_len; k++) {
                    ESP_LOG_BUFFER_HEXDUMP("processCatMessages", &read_buffer[k], 1, ESP_LOG_INFO);
                    //ESP_LOGI("processCatMessages", ",");
                }
                ESP_LOGI("processCatMessages", " msg_len = %d END", msg_len);
            #endif
            if (read_buffer[0] == START_BYTE && read_buffer[1] == START_BYTE) {
                radio_address_received = read_buffer[3];
                //if (read_buffer[3] == radio_address) {
                if (read_buffer[3] != 0) {
                    if (read_buffer[2] == CONTROLLER_ADDRESS || read_buffer[2] == BROADCAST_ADDRESS) {

                        for (cmd_num = CIV_C_F_SEND; cmd_num < End_of_Cmd_List; cmd_num++)  // loop through the command list structure looking for a pattern match
                        {
                            //ESP_LOGI("processCatMessages", "processCatMessageslist: list index = "); DPRINTLN(cmd_num);
                            for (i = 1; i <= cmd_List[cmd_num].cmdData[0]; i++)  // start at the highest and search down. Break out if no match. Make it to the bottom and you have a match
                            {
                                //ESP_LOGI("processCatMessages", "processCatMessages: byte index = "); DPRINTLN(i);
                                //ESP_LOGI(TAG,"processCatMessages: cmd_num=%d from radio, current byte from radio = %X  next byte=%X, on remote length=%d and cmd=%X",cmd_num, read_buffer[3+i], read_buffer[3+i+1], cmd_List[cmd_num].cmdData[0], cmd_List[cmd_num].cmdData[1]);
                                if (cmd_List[cmd_num].cmdData[i] != read_buffer[3 + i]) {
                                //ESP_LOGI("processCatMessages", "processCatMessages: Skip this one - Matched 1 element: look at next field, if any left. CMD Body Length = ");
                                //ESP_LOGI("processCatMessages", cmd_List[cmd_num].cmdData[0]); DPRINTF(" CMD  = "); ESP_LOGI(TAG,(cmd_List[cmd_num].cmdData[i], HEX);DPRINTF(" next RX byte = "); DPRINTLN(read_buffer[3+i+1],HEX);
                                match = 0;
                                break;
                                }
                                match++;
                                //ESP_LOGI("processCatMessages", "processCatMessages: Possible Match: Len = "); ESP_LOGI(TAG,(cmd_List[cmd_num].cmdData[0],DEC); DPRINTF("  CMD1 = "); ESP_LOGI(TAG,(read_buffer[4],HEX);
                                //ESP_LOGI("processCatMessages", " CMD2  = "); ESP_LOGI(TAG,(read_buffer[5],HEX); DPRINTF(" Data1/Term  = "); DPRINTLN(read_buffer[6],HEX);
                            }

                            //if (read_buffer[3+i] == STOP_BYTE)  // if the next byte is not a stop byte then it is thge next cmd byte or maybe a data byte, depends on cmd length

                            if (match && (match == cmd_List[cmd_num].cmdData[0])) 
                            {
                                //ESP_LOGI("processCatMessages", "processCatMessages:    FOUND MATCH: Len = "); ESP_LOGI(TAG,(cmd_List[cmd_num].cmdData[0],DEC); DPRINTF("  CMD1 = "); ESP_LOGI(TAG,(read_buffer[4],HEX);
                                //ESP_LOGI("processCatMessages", " CMD2  = "); ESP_LOGI(TAG,(read_buffer[5],HEX);  DPRINTF(" Data1/Term  = "); ESP_LOGI(TAG,(read_buffer[6],HEX); DPRINTF("  Message Length = "); DPRINTLN(msg_len);
                                break;
                            }
                        }

                        data_start_idx = 4 + cmd_List[cmd_num].cmdData[0];
                        data_len = msg_len - data_start_idx - 1;

                        //uint8_t k;
                        //for (k = 0; k < msg_len; k++)
                        //    CIV_data[k] = read_buffer[data_start_idx + k];

                        ESP_LOGI("processCatMessages", "cmd = %X  data_start_idx = %d  data_len = %d", cmd_List[cmd_num].cmdData[1], data_start_idx, data_len);

                        if (cmd_num >= End_of_Cmd_List - 1) 
                        {
                            ESP_LOGI("processCatMessages", "processCatMessages: No message match found - cmd_num = %d  read_buffer[4 & 5] = %X %X", cmd_num, read_buffer[4], read_buffer[5]);
                            //knowncommand = false;
                        } 
                        else 
                        {
                            ESP_LOGI("processCatMessages", "Call CIV_Action");
                            CIV_Action(cmd_num, data_start_idx, data_len, msg_len, read_buffer);
                        }
                    }  // is controller address
                }    // is radio address
            }      // is preamble
        }        // readline
    }          // while
}

//
//    formatVFO()
//
char *formatVFO(uint64_t vfo) {
  static char vfo_str[20] = { "" };
  //if (ModeOffset < -1 || ModeOffset > 1)
  //vfo += ModeOffset;  // Account for pitch offset when in CW mode, not others

  uint32_t MHz = (vfo / 1000000 % 1000000);
  uint16_t Hz = (vfo % 1000) / 10;
  uint16_t KHz = ((vfo % 1000000) - Hz) / 1000;
  if (board_type == M5ATOMS3) {
    sprintf(vfo_str, "%lu.%03u.%01u", MHz, KHz, Hz/10);
  } else {
    sprintf(vfo_str, "%lu.%03u.%02u", MHz, KHz, Hz);
  }
  ///sprintf(vfo_str, "%-13s", "012345.123.123");  // 999GHZ max  47G = 47000.000.000
  ///DPRINT("New VFO: ");DPRINTLN(vfo_str);
  return vfo_str;
}

void draw_new_screen(void) {
    int16_t x = 46;  // start position
    int16_t y = 16;
    int16_t w = 319;  // end of a line
    int16_t y1 = y + 13;
    //int16_t h = 20;
    int16_t font_sz = 4;  // font size
    //ESP_LOGI(TAG,"+++++++++draw new screen");

    #ifdef  ATOMS3
        font_sz = 3;  // downsize for Atom
        x = 46;  // start position
        y = 6;
        w = 127;  // end of a line
        y1 = 14;   // height of line
        //h = 12;

        int16_t color = TFT_YELLOW;
        M5.Lcd.fillScreen(TFT_BLACK);
        M5.Lcd.setTextColor(TFT_YELLOW, background_color);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
        M5.Lcd.setTextDatum(MC_DATUM);
        //M5.Lcd.drawString("CI-V band Decoder", (int)(M5.Lcd.width() / 2), y, font_sz);
        M5.Lcd.drawString(title, (int)(M5.Lcd.width() / 2), y, font_sz);
        M5.Lcd.drawFastHLine(1, y1, w, TFT_RED);  // separator below title
        M5.Lcd.setTextDatum(MC_DATUM);
        M5.Lcd.setTextColor(TFT_CYAN, background_color);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
    #endif
    
    // write the Band and PTT icons
    display_Freq(frequency, true);
    display_PTT(PTT, true);
    display_Band(band, true);  // true means draw the icon regardless of state
    display_Time(UTC, true);
    display_Grid(Grid_Square, true);
}

//  _UTC does nothing now but can be used to change a future clock label
void display_Time(uint8_t _UTC, bool _force) {
    #ifdef POLL_FOR_TIME
        static uint64_t time_last_disp_UTC = esp_timer_get_time();
        
        if ((esp_timer_get_time() >= time_last_disp_UTC + POLL_RADIO_UTC*1000) || _force) {
            if (USBH_connected) // wait until we are fully running
                poll_for_time();
            int x = 10;
            int x1 = 310;
            int y = 52;
            int font_sz = 4;

            #if !defined (M5STAMPC3U) && defined (ATOMS3)
                if (board_type == M5ATOMS3) {
                    font_sz = 3;  // downsize for Atom
                    x = 1;
                    x1 = 127;
                    y = 25;
                    font_sz = 3;
                }
                
                char temp_t[64] = {};
                //char *myDate = ctime(&t);   for testing
                //ESP_LOGI(TAG, "myDate: %s now %lld", myDate, t);

                struct tm local = *localtime(&t);
                ESP_LOGI(TAG, "Time is now : %02d/%02d/%02d    %02d:%02d:%02d", (local.tm_mon)+1, local.tm_mday, \
                    (local.tm_year+1900), local.tm_hour, local.tm_min, local.tm_sec);

                localtime_r(&t, &tm);
                
                //sprintf(temp_t,"%d-%02d-%02d, %02d:%02d:%02d",(tm.tm_year)+1900,(  tm.tm_mon)+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec );
                //ESP_LOGI("display_Time", "%s", temp_t);      
                if ((tm.tm_year+1900) != 1970) {   // skip if date not updated yet which shows 1970
                    M5.Lcd.setTextColor(TFT_LIGHTGREY, background_color);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
                    M5.Lcd.setTextDatum(ML_DATUM);  // x is left side
                    sprintf(temp_t, "%02d/%02d/20%02d", (tm.tm_mon)+1, tm.tm_mday, (tm.tm_year)+1900);
                    M5.Lcd.drawString(temp_t, x, y, font_sz);
                    ESP_LOGI("display_Time","Date is now %s", temp_t);
                    M5.Lcd.setTextDatum(MR_DATUM);  // x1 is right side
                    sprintf(temp_t, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
                    M5.Lcd.drawString(temp_t, x1, y, font_sz);
                    ESP_LOGI("display_Time","Time is now %s", temp_t);
                }
            #endif

            time_last_disp_UTC = esp_timer_get_time();
        }
    #endif
}

void display_PTT(bool _PTT_state, bool _force) {
    static bool _prev_PTT_state = true;
    char PTT_Tx[3] = "TX";
    int x = 310;
    int y = 150;
    int x1 = x - 33;  // upper left corner of outline box
    int y1 = y - 18;
    int font_sz = 4;  // font size
    int w = 38;       // box width
    int h = 30;       // box height
    int r = 4;        // box radius corner size

    #if !defined (M5STAMPC3U) &&  defined (ATOMS3)
        M5.Lcd.setTextDatum(MR_DATUM);
        if (board_type == M5ATOMS3) {
        font_sz = 3;  // downsize for Atom
        x = 124;
        y = 118;
        x1 = x-16;
        y1 = y-8; 
        w = 20;
        h = 14;
        r = 4;
        }
        
        if (_PTT_state != _prev_PTT_state || _force) {
        #ifdef PRINT_PTT_TO_SERIAL
            ESP_LOGI(TAG, "*********************************************** PTT = %d", _PTT_state);
        #endif
        if (_PTT_state) {
            M5.Lcd.fillRoundRect(x1, y1, w, h, r, TFT_RED);
            M5.Lcd.setTextColor(TFT_WHITE);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
            M5.Lcd.drawString(PTT_Tx, x, y, font_sz);
        } else {
            M5.Lcd.fillRoundRect(x1, y1, w, h, r, background_color);
            M5.Lcd.setTextColor(TFT_RED);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
            M5.Lcd.drawString(PTT_Tx, x, y, font_sz);
        }
        M5.Lcd.drawRoundRect(x1, y1, w, h, r, TFT_RED);
        _prev_PTT_state = _PTT_state;
        }
    #endif
}

void display_Freq(uint64_t _freq, bool _force) {
  static uint64_t _prev_freq;
  int16_t x = 1;  // start position
  int16_t y = 104;
  int16_t font_sz = 6;  // font size

  if ((_freq != _prev_freq && _freq != 0) || _force) {
    #ifdef PRINT_VFO_TO_SERIAL
      ESP_LOGI(TAG,"VFOA: %13sMHz - Band: %s  Mode: %s  DataMode: %s  Filter: %s  Source: BLE %d, USBHost %d, BTClassic %d", formatVFO(_freq), bands[band].band_name, \
          modeList[bands[band].mode_idx].mode_label, ModeStr[bands[band].datamode], FilStr[bands[band].filt], BLE_connected, USBH_connected, btConnected);
    #endif
    if (board_type == M5ATOMS3) {
      font_sz = 4;  // downsize for Atom
      y = 54;
    }

    #if !defined (M5STAMPC3U) &&  defined (ATOMS3)
      int16_t color = TFT_WHITE;
      //M5.Lcd.fillRect(x, y, x1, y1, background_color);
      M5.Lcd.setTextDatum(MC_DATUM);
      M5.Lcd.setTextColor(background_color, background_color);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
      M5.Lcd.setTextColor(background_color);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
      M5.Lcd.drawString(formatVFO(_prev_freq), (int)(M5.Lcd.width() / 2), y, font_sz);
      //M5.Lcd.setTextColor(color, background_color);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
      M5.Lcd.setTextColor(color);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
      M5.Lcd.drawString(formatVFO(_freq), (int)(M5.Lcd.width() / 2), y, font_sz);
    #endif // M5STAMPC3U
    _prev_freq = _freq;
  }
}

void display_Band(uint8_t _band, bool _force) {
  static uint8_t _prev_band = 255;
  int x = 8;
  int y = 150;
  int font_sz = 4;


  if (_band != _prev_band || _force) {
    // Update our outputs
    //Band_Decode_Output(band, false);
    //sendBand(band);   // change the IO pins to match band
    //ESP_LOGI(TAG,"Band %s", bands[_band].band_name);

    #if !defined (M5STAMPC3U) &&  defined (ATOMS3)
      if (board_type == M5ATOMS3) {
        font_sz = 4;  // downsize for Atom
        x = 1;
        y= 118;
      }
      M5.Lcd.setTextDatum(ML_DATUM);
      M5.Lcd.setTextDatum(ML_DATUM);
      M5.Lcd.setTextColor(background_color, background_color);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
      M5.Lcd.drawString(bands[_prev_band].band_name, x, y, font_sz);
      M5.Lcd.setTextColor(TFT_CYAN);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
      M5.Lcd.drawString(bands[_band].band_name, x, y, font_sz);
    #endif

    if (frequency != 0) {
      if (!update_radio_settings_flag)
        #ifdef SD_CARD
          write_bands_data();  // save on band changes.  Other times would be good bu this catches the most.
        #else
        ; // nothing
        #endif
      else                   // by this time we should be stable after XVTR transitions
        update_radio_settings_flag = false;
    }
    _prev_band = _band;
  }
}

void display_Grid(char _grid[], bool _force) {
  static char _last_grid[9] = {};
  int x = 8;
  int y = 184;
  int font_sz = 4;
  // call to convert the strings for Lat and long fronm CIV to floats and then caluclate grid
  if ((strcmp(_last_grid, _grid)) || _force) {
    //ESP_LOGI(TAG,"Grid Square = %s",_grid);

    #if !defined (M5STAMPC3U) &&  defined (ATOMS3)
      if (board_type == M5ATOMS3) {
        font_sz = 4;  // downsize for Atom
        y = 86;
        M5.Lcd.setTextDatum(MC_DATUM);
        M5.Lcd.setTextColor(background_color, background_color);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
        M5.Lcd.drawString(_last_grid, (int)(M5.Lcd.width() / 2), y, font_sz);
        M5.Lcd.setTextColor(TFT_DARKGREEN, background_color);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
        M5.Lcd.drawString(_grid, (int)(M5.Lcd.width() / 2), y, font_sz);
      } else {
        M5.Lcd.setTextDatum(ML_DATUM);
        M5.Lcd.setTextColor(background_color, background_color);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
        M5.Lcd.drawString(_last_grid, x, y, font_sz);
        M5.Lcd.setTextColor(TFT_GREEN, background_color);  //Set the color of the text from 0 to 65535, and the background color behind it 0 to 65535
        M5.Lcd.drawString(_grid, x, y, font_sz);
      }
    #endif   
    strcpy(_last_grid, _grid);
  }
}

void poll_for_time(void){
    if (init_done && !sending) { // && board_type == M5ATOMS3) {
        /*   Do not need to get UTC offset since it is called at startup and is a global.
        if (radio_address == IC905)                  //905
            sendCatRequest(CIV_C_UTC_READ_905, 0, 0);  //CMD_READ_FREQ);
        else if (radio_address == IC705)             // 705
            sendCatRequest(CIV_C_UTC_READ_705, 0, 0);  //CMD_READ_FREQ);
        vTaskDelay(pdMS_TO_TICKS(2));
        */
        ESP_LOGI(TAG, "***Get Time from radio");
        sendCatRequest(CIV_C_MY_POSIT_READ, 0, 0); 
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void refresh_display(void) {
    display_Freq(frequency, false);
    #ifndef USE_LEDS
        display_Time(UTC, false);
        display_PTT(PTT, false);
        display_Band(band, false);  // true means draw the icon regardless of state
        display_Grid(Grid_Square, false);
    #endif  // If we have LEDs then we likey have no screen to draw.
}

uint8_t Get_Radio_address(void) {
    uint8_t retry_Count = 0;
    
    if (USBH_connected)
    {
        while (radio_address == 0x00 || radio_address == 0xFF || radio_address == 0xE0) {
            if (radio_address_received == 0) {
                ESP_LOGI("Get_Radio_address:", "Radio not found - retry count = %X", retry_Count);
                vTaskDelay(100);
                if (retry_Count++ > 4)
                    break;
            } else {
                radio_address = radio_address_received;
                ESP_LOGI("Get_Radio_address: ", "Radio found at %X", radio_address);
            }
        }
    }
    return retry_Count;
}

void setup_IO(void)
{   
    // Set up an interrupt on our PTT input pin and set output pins
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g. GPIO_BAND_OUTPUT_144 and GPIO_PTT_OUTPUT_144
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = (gpio_pulldown_t) 0;
    //disable pull-up mode
    io_conf.pull_up_en = (gpio_pullup_t) 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of falling edge (may be rising edge after input is buffered, thus inverted)
    //io_conf.intr_type = GPIO_INTR_NEGEDGE;  // or POSEDGE
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO_PTT_INPUT pin
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = (gpio_pullup_t) 1;
    gpio_config(&io_conf);

    // De-Glitch PTT
    gpio_glitch_filter_handle_t filter;

    //gpio_flex_glitch_filter_config_t filter_config = {GLITCH_FILTER_CLK_SRC_DEFAULT, GPIO_PTT_INPUT, 5000 , 2000};
    //ESP_ERROR_CHECK(gpio_new_flex_glitch_filter(&filter_config, &filter));
    
    gpio_pin_glitch_filter_config_t filter_config = {GLITCH_FILTER_CLK_SRC_DEFAULT, (gpio_num_t) GPIO_PTT_INPUT};
    ESP_ERROR_CHECK(gpio_new_pin_glitch_filter(&filter_config, &filter));
    
    ESP_ERROR_CHECK(gpio_glitch_filter_enable(filter));

    //change gpio intrrupt type for one pin
    //gpio_set_intr_type(GPIO_PTT_INPUT, GPIO_INTR_ANYEDGE);

    // Set output pins to low.  High turns on the buffers (inverted)
    PTT_Output(0,0);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_PTT_Input, "gpio_PTT_Input", 8092, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add((gpio_num_t)GPIO_PTT_INPUT, gpio_isr_handler, (void*) (gpio_num_t) GPIO_PTT_INPUT);

    // Set up ADC for LED brightness dimming
    
    //-------------ADC2 Init---------------//
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    //-------------ADC2 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC2_CHAN0, &config));
}

/**
 * @brief Main application
 *
 * Here we open a USB CDC device and send some data to it
 */
//int app_main(void)   // for .c files
extern "C" void app_main(void)  // for .cpp files
{
    ESP_LOGI(TAG, "IC-905 USB Band Decoder and PTT Breakout - K7MDL Jan 2025");

    setup_IO();

    #ifdef USE_LEDS
        ledc_init();    
    #else  // USE RGB pin 48 or external LED via GPIO for the red PTT in LED on pin 47
        gpio_reset_pin(GPIO_NUM_48);
        gpio_set_direction(GPIO_NUM_48, GPIO_MODE_OUTPUT_OD);  // using RGB LED as simple PTT LED
        gpio_set_level(GPIO_NUM_48, 1);  // Turn it Off.
    #endif 

    #ifdef USB_PC

        // Create FreeRTOS primitives
        app_queue = xQueueCreate(5, sizeof(app_message_t));
        assert(app_queue);
        
        ESP_LOGI(TAG, "USB initialization");
        const tinyusb_config_t tusb_cfg = {
            .device_descriptor = NULL,
            .string_descriptor = NULL,
            .external_phy = false,
        #if (TUD_OPT_HIGH_SPEED)
            .fs_configuration_descriptor = NULL,
            .hs_configuration_descriptor = NULL,
            .qualifier_descriptor = NULL,
        #else
            .configuration_descriptor = NULL,
        #endif // TUD_OPT_HIGH_SPEED
            .self_powered = NULL,                        /*!< This is a self-powered USB device. USB VBUS must be monitored. */
            .vbus_monitor_io = NULL                       /*!< GPIO for VBUS monitoring. Ignored if not self_powered. */
        };

        ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

        tinyusb_config_cdcacm_t acm_cfg = {
            .usb_dev = TINYUSB_USBDEV_0,
            .cdc_port = TINYUSB_CDC_ACM_0,
            .rx_unread_buf_sz = 64,
            .callback_rx = tinyusb_cdc_rx_callback, // the first way to register a callback
            .callback_rx_wanted_char = NULL,
            .callback_line_state_changed = NULL,
            .callback_line_coding_changed = NULL
        };

        ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
        /* the second way to register a callback */
        ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                            TINYUSB_CDC_ACM_0,
                            CDC_EVENT_LINE_STATE_CHANGED,
                            &tinyusb_cdc_line_state_changed_callback));

        #if (CONFIG_TINYUSB_CDC_COUNT > 1)
        acm_cfg.cdc_port = TINYUSB_CDC_ACM_1;
        ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
        ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                            TINYUSB_CDC_ACM_1,
                            CDC_EVENT_LINE_STATE_CHANGED,
                            &tinyusb_cdc_line_state_changed_callback));
        #endif

        ESP_LOGI(TAG, "USB initialization DONE");

        if (xQueueReceive(app_queue, &msg, portMAX_DELAY)) {
            if (msg.buf_len) {

                /* Print received data*/
                ESP_LOGI(TAG, "Data from channel %d:", msg.itf);
                ESP_LOG_BUFFER_HEXDUMP(TAG, msg.buf, msg.buf_len, ESP_LOG_INFO);

                /* write back */
                tinyusb_cdcacm_write_queue(msg.itf, msg.buf, msg.buf_len);
                esp_err_t err = tinyusb_cdcacm_write_flush(msg.itf, 0);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "CDC ACM write flush error: %s", esp_err_to_name(err));
                }
            }
        }
    #endif

    #ifdef CONFIG_IDF_TARGET_ESP32S3
        #if !defined (M5STAMPC3U) &&  defined (ATOMS3)
        auto cfg = M5.config();
        M5.begin(cfg);
        M5.Power.begin();
        #ifdef ATOMS3
            ESP_LOGI(TAG,"ATOMS3 defined");
        #endif
        board_type = M5.getBoard();
        if (board_type == M5ATOMS3) {
        ESP_LOGI(TAG,"AtomS3 ext i2c pins defined");
        //Wire.begin(2,1);   // M5AtomS3 external i2c
        M5.Lcd.setRotation(rotation);  // 0 to 3 rotate, 4 to 7 reverse and rotate.
        M5.Lcd.setBrightness(brightness);   // 0- 255
        } else {
        ESP_LOGI(TAG,"CoreS3 or CoreS3SE ext i2C pins defined");
        //Wire.begin(12,11);   // CoreS3 and ?StampC3U?
        }
        draw_new_screen();
        #endif
    #endif

    // create a new rtos task for main radio control loop -- not in use yet
    BaseType_t loop_task_created = xTaskCreate(usb_loop_task, "usb_loop", 4096, xTaskGetCurrentTaskHandle(), USB_HOST_PRIORITY, NULL);
    assert(loop_task_created == pdTRUE);

    // Setup our USB Host port
    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem);

    // Install USB Host driver. Should only be called once in entire application
    ESP_LOGI(TAG, "Installing USB Host");
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .root_port_unpowered = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .enum_filter_cb = NULL
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Create a task that will handle USB library events
    BaseType_t task_created = xTaskCreate(usb_lib_task, "usb_lib", 8092, xTaskGetCurrentTaskHandle(), USB_HOST_PRIORITY, NULL);
    assert(task_created == pdTRUE);

    ESP_LOGI(TAG, "Installing CDC-ACM driver");
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 1000,
        .out_buffer_size = 512,
        .in_buffer_size = 512,
        .event_cb = handle_event,
        .data_cb = handle_rx,
        .user_arg = NULL
    };

    while (true) {
        //cdc_acm_dev_hdl_t cdc_dev = NULL;

        // Open USB device from tusb_serial_device example example. Either single or dual port configuration.
        ESP_LOGI(TAG, "Opening CDC ACM device 0x%04X:0x%04X...", CDC_HOST_ANY_VID, CDC_HOST_ANY_PID);
       
       esp_err_t err = cdc_acm_host_open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID, 0, &dev_config, &cdc_dev);
        //esp_err_t err = cdc_acm_host_open(USB_DEVICE_VID, USB_DEVICE_PID, 0, &dev_config, &cdc_dev);
       
        if (ESP_OK != err) {
            //ESP_LOGI(TAG, "Opening CDC ACM device 0x%04X:0x%04X...", USB_DEVICE_DUAL_VID, USB_DEVICE_DUAL_PID);
            //err = cdc_acm_host_open(USB_DEVICE_DUAL_VID, USB_DEVICE_DUAL_PID, 0, &dev_config, &cdc_dev);
            if (ESP_OK != err) {
                ESP_LOGI(TAG, "Failed to open device");
                continue;
            }
        }
        cdc_acm_host_desc_print(cdc_dev);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Test sending and receiving: responses are handled in handle_rx callback
        //ESP_LOGI(TAG, "Test Send to target");
        //ESP_ERROR_CHECK(cdc_acm_host_data_tx_blocking(cdc_dev, (const uint8_t *)EXAMPLE_TX_STRING, strlen(EXAMPLE_TX_STRING), EXAMPLE_TX_TIMEOUT_MS));
        
        // Test Line Coding commands: Get current line coding, change it 9600 7N1 and read again
        ESP_LOGI(TAG, "Setting up line coding");
        cdc_acm_line_coding_t line_coding;
        ESP_ERROR_CHECK(cdc_acm_host_line_coding_get(cdc_dev, &line_coding));
        ESP_LOGI(TAG, "Line Get: Rate: %"PRIu32", Stop bits: %"PRIu8", Parity: %"PRIu8", Databits: %"PRIu8"",
                 line_coding.dwDTERate, line_coding.bCharFormat, line_coding.bParityType, line_coding.bDataBits);

        line_coding.dwDTERate = 115200;
        line_coding.bDataBits = 8;
        line_coding.bParityType = 1;
        line_coding.bCharFormat = 1;
        ESP_ERROR_CHECK(cdc_acm_host_line_coding_set(cdc_dev, &line_coding));
        ESP_LOGI(TAG, "Line Set: Rate: %"PRIu32", Stop bits: %"PRIu8", Parity: %"PRIu8", Databits: %"PRIu8"",
                 line_coding.dwDTERate, line_coding.bCharFormat, line_coding.bParityType, line_coding.bDataBits);

        ESP_ERROR_CHECK(cdc_acm_host_line_coding_get(cdc_dev, &line_coding));
        ESP_LOGI(TAG, "Line Get: Rate: %"PRIu32", Stop bits: %"PRIu8", Parity: %"PRIu8", Databits: %"PRIu8"",
                line_coding.dwDTERate, line_coding.bCharFormat, line_coding.bParityType, line_coding.bDataBits);
      
        ESP_ERROR_CHECK(cdc_acm_host_set_control_line_state(cdc_dev, false, false));
        
        USBH_connected = true;

        // We can now send to CI-V
        vTaskDelay(pdMS_TO_TICKS(200));  // Time for radio to to be ready for comms after connection
        // Have observed a mode message will be received on connection so awit for that to finish.
        
        ESP_LOGI(TAG, "***Starting CI-V communications");
        //SetFreq(145500000);  // Used for testing when there is no screen, can see radio respond to something.
        vTaskDelay(pdMS_TO_TICKS(200));
        
        while (radio_address == 0 || frequency == 0) {
            ESP_LOGI(TAG, "***Get frequency from radio");
            sendCatRequest(CIV_C_F_READ, 0, 0);  // Get current VFO     
            vTaskDelay(pdMS_TO_TICKS(100));

            ESP_LOGI(TAG, "***Get CI-V address from radio");
            Get_Radio_address();
            vTaskDelay(pdMS_TO_TICKS(100));  // Time for radio to to beready for comms after connection
        }
        
        // Get started by retrieving frequency, mode, time & location and time offset.
        ESP_LOGI(TAG, "***Get extended mode info from radio");
        sendCatRequest(CIV_C_F26A, 0, 0);  // Get extended info -  mode, filter, and datamode status
        vTaskDelay(pdMS_TO_TICKS(20));
        
        ESP_LOGI(TAG, "***Get UTC Offset from radio");
        if (radio_address == IC905)                  //905
            sendCatRequest(CIV_C_UTC_READ_905, 0, 0);  //CMD_READ_FREQ);
        else if (radio_address == IC705)             // 705
            sendCatRequest(CIV_C_UTC_READ_705, 0, 0);  //CMD_READ_FREQ);
        vTaskDelay(pdMS_TO_TICKS(20));
        
        ESP_LOGI(TAG, "***Get time, date and position from radio.  We will then calculate grid square");
        sendCatRequest(CIV_C_MY_POSIT_READ, 0, 0);  //CMD_READ_FREQ);
        vTaskDelay(pdMS_TO_TICKS(20));

        #ifdef PC_PASSTHROUGH
            esp_log_level_set("*", ESP_LOG_NONE);
        #endif

        init_done = 1;

        ESP_LOGI(TAG, "***Now wait for radio dial and band change messages");
        // usb_loop_task() is where our program runs within now.  
        // handler_rx takes care of received events
        // Do not Transmit to USB (such as call send_CAT_Request()) from the handler_rx, they will overlap and cause timeouts.
        // Incoming CIV data appears in handler_rx which in turn calls processmessages() and CIV_Action(). 
        // If anything needs to be TX back to the radio by those processes like mode calling get ext mode, 
        // or read frequency() calling get ext mode, set a flag and let usb_loop_task() handle it.
        
        // We are done.  The tasks are running things now. Wait for device disconnection and start over
        
        // program waits here while tasks run handling events
        xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);
    }
}
