/*
 * IC905_ESP32-S3_PTT_Breakout.c
 *
 * This program is a USB Host Serial device that reads the Icom IC-905 USB CI_V frequency message #00.
 * For 10GHz and higher the IC-905 adds 1 extra byte thqat must be accounted for to decod the BCD  encoded frequency
 * The last data byte sent for frequency is the MSB.
 * There are other potential messages that contain frequnecy that can be leveraged also.
 * the radio PTT (aka SEND output) is wired into this CPU and monitored.
 * There are 6 PTT and 6 band enable outputs, with status LEDs for each
 * PTT is normally +5V if directly connected to the SEND jack with a weak pullup at the radio and/or at the CPU
 * When PPT = 0 (radio is in TX) then we look at the last frequency seen and send 0 to the matching band's PTT output
 * 
 *  Future An optional display (OLED likely) for output status, frequency, time from radio, and caclulate 8 or
 *  even 10 digit grid square from thhe radio CIV Time and offset messages as is done in teh 705 and 905 full
 *  band decoder projects for ESP32 and the Teensy 4.   For the 705 3-band transverter box project, I am using
 *  a M5StampC3U with an i2c conencted 0.91" OLED 128x32 and it works well.  This data set requires polling the radio.
 * 
 *  ---------------------------------------------------------------------------------
 * The USB Serial Host part of this program is based on cdc_acm_host ezxample file from 
 * 
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

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
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "IC905_ESP32-S3_PTT_Breakout.h"
#include "Decoder.h"
#include "CIV.h"
#include "driver/i2c_master.h"
#include "ssd1306.h"

struct Bands bands[NUM_OF_BANDS] = {
  { "DUMMY", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF },                                       // DUMMY Band to avoid using 0
  { "2M", 144000000, 148000000, 0, 144200000, 1, 1, 0, 1, 0, 0, 0, DECODE_INPUT_BAND144 },                  // 2m
  { "70cm", 430000000, 450000000, 0, 432100000, 1, 1, 0, 1, 0, 0, 0, DECODE_INPUT_BAND430 },                // 430/440  No LO
  { "23cm", 1240000000, 1300000000, 0, 1296100000, 1, 1, 0, 1, 0, 0, 0, DECODE_INPUT_BAND1200 },            // 1296Mhz  with 144Mhz LO
  { "13cm", 2300000000, 2450000000, 0, 2304100000, 1, 1, 0, 1, 0, 0, 0, DECODE_INPUT_BAND2300 },            // 2.3 and 2.4GHz
  { "6cm",  5650000000, 5925000000, 0, 5760100000, 1, 1, 0, 1, 0, 0, 0, DECODE_INPUT_BAND5600 },            // 5.7GHz
  { "3cm", 10000000000, 10500000000, 0, 10368100000, 1, 1, 0, 1, 0, 0, 0, DECODE_INPUT_BAND10G },           // 10GHz
};

char title[25] = "IC-905 CI-V Band Decoder";
uint16_t baud_rate;                   //Current baud speed
uint32_t readtimeout = 10;            //Serial port read timeout
uint8_t band = BAND_2M;
//uint16_t background_color = TFT_BLACK;
bool PTT = false;
bool prev_PTT = true;
extern char Grid_Square[];
extern struct cmdList cmd_List[];
uint8_t radio_address = RADIO_ADDR;  //Transceiver address.  0 allows auto-detect on first messages form radio
bool auto_address = false;           // If true, detects new radio address on connection mode changes
                                     // If false, then the last used address, or the preset address is used.
                                     // If Search for Radio button pushed, then ignores this and looks for new address
                                     //   then follows rules above when switch connections
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
void display_Freq(uint64_t _freq, bool _force);
void display_Time(uint8_t _UTC, bool _force);
void display_PTT(bool _PTT_state, bool _force);
void display_Band(uint8_t _band, bool _force);
void display_Grid(char _grid[], bool _force);
void SetFreq(uint64_t Freq);
uint8_t getBand(uint64_t _freq);
void read_Frequency(uint64_t freq, uint8_t data_len);
uint8_t formatFreq(uint64_t vfo, uint8_t vfo_dec[]);
extern void CIV_Action(const uint8_t cmd_num, const uint8_t data_start_idx, const uint8_t data_len, const uint8_t msg_len, const uint8_t rd_buffer[]);
uint8_t pass_PC_to_radio(void);
extern uint8_t USBHost_ready;  // 0 = not mounted.  1 = mounted, 2 = system not initialized
bool USBH_connected = false;
extern uint16_t background_color;
uint64_t frequency;
extern bool update_radio_settings_flag;
static const char *TAG = "USB-CDC";
static SemaphoreHandle_t device_disconnected_sem;
cdc_acm_dev_hdl_t cdc_dev;
uint8_t *r = read_buffer;
bool msg_done_flag;
bool get_ext_mode_flag = false;
cdc_acm_dev_hdl_t cdc_dev = NULL;

// Mask all 12 of our Band and PTT output pins for Output mode
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_BAND_OUTPUT_144 | 1ULL<<GPIO_BAND_OUTPUT_430 \
    | 1ULL<<GPIO_BAND_OUTPUT_1200 | 1ULL<<GPIO_BAND_OUTPUT_2300 | 1ULL<<GPIO_BAND_OUTPUT_5600 | 1ULL<<GPIO_BAND_OUTPUT_10G \
    | 1ULL<<GPIO_PTT_OUTPUT_144 | 1ULL<<GPIO_PTT_OUTPUT_430 | 1ULL<<GPIO_PTT_OUTPUT_1200 \
    | 1ULL<<GPIO_PTT_OUTPUT_2300 | 1ULL<<GPIO_PTT_OUTPUT_5600 | 1ULL<<GPIO_PTT_OUTPUT_10G)

// Mask our 1 PTT input IO pin for Input with pull up. An interrupt wil be assigned to this pin.
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_PTT_INPUT)
#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;

/**
 * Brief:
 * This GPIO code configures gpio outputs and gpio interrupt on the PTT GPIO input
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
            PTT = gpio_get_level(io_num);  // Invert for buffer
            printf("GPIO[%lu] intr, val: %d\n", io_num, PTT);
            PTT_Output(band, !PTT);
        }
    }
}

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
        printf("*");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief USB Host Radio TX queue handling task
 *
 * @param arg Unused
 */
static void usb_TX_task(void *arg)
{
    while (1) {  
        printf("+");
        if (get_ext_mode_flag) {
            ESP_LOGI(TAG, "***Get extended mode info from radio");
            sendCatRequest(CIV_C_F26A, 0, 0);  // Get extended info -  mode, filter, and datamode status
            get_ext_mode_flag = false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

 void printBinaryWithPadding(uint8_t num, char bin_num[9]) { 
    for (int i = sizeof(uint8_t) * 8 - 1; i >= 0; i--) {
        char b = ((num >> i) & 1) + 0x30;
        //printf("%c", b);
        bin_num[7-i] = b;
        //if (i % 4 == 0) printf(" "); // Group by 4 bits for readability
    }
    bin_num[8] = '\0';
    //printf("binary is %s\n", bin_num);
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

    for (uint8_t k = 0; k <= msg_len; k++) {
        ESP_LOGI("sendCatRequest --> Tx Raw Msg: ", "%X,", req[k]);
    }
    ESP_LOGI("sendCatRequest", " msg_len = %d   END", msg_len);

    if (USBH_connected) {
        if (msg_len < sizeof(req) - 1) {  // ensure our data is not longer than our buffer
            //ESP_LOGI("sendCatRequest", "***Send CI-V Msg: ");
            ESP_LOGI("Send_CatRequest", "*** Send Cat Request Msg - result = %d", cdc_acm_host_data_tx_blocking(cdc_dev, (const uint8_t *)req, msg_len+1, 1000));
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_LOGI("sendCatRequest", " END TX MSG, msg_len = %d", msg_len);
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
        //Serial.printf("read_Frequency: Last Freq %-13llu\n", frequency);
        if (!update_radio_settings_flag) {   // wait until any XVTR transition complete
        bands[band].VFO_last = frequency;  // store Xvtr or non-Xvtr band displayed frequency per band before it changes.
        prev_band = band;                  // store associated band index
        }
    }  // if an Xvtr band, subtract the offset to get radio (IF) frequency

    // Could do more validation here. Freq Calculation moved to CIV.cpp
    frequency = freq;
    
    band = getBand(frequency);
    if (band != prev_band) {
        prev_band = band;
        ESP_LOGI("read_Frequency", "***Get extended mode data from radio after band change - Setting flag");
        get_ext_mode_flag = true;
    }

    // Use the band to operate our band enable outputs for the 6 905 bands.
    Band_Decode_Output(band);

    //Serial.printf("read_Frequency: Freq %-13llu  band =  %d  Xvtr_Offset = %llu  datalen = %d   btConnected %d   USBH_connected %d   BT_enabled %d   BLE_connected %d  radio_address %X\n", frequency, band, bands[XVTR_Band].Xvtr_offset, data_len, btConnected, USBH_connected, BT_enabled, BLE_connected, radio_address);
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
  return 0xFF;  // no band for considered frequency found
}

// Send new frequency to radio, radio will change bands as needed.
// ToDo:  Radio mode and other settings are not touched so stay the same as the last band used.  We are only changing the frequency, nothing else.
//        Need to save mode, filter and other stuff to return each band to the last way it was used.
void SetFreq(uint64_t Freq) {
  uint8_t vfo_dec[7] = {};
  uint8_t len = formatFreq(Freq, vfo_dec);  // Convert to BCD string
  ESP_LOGI("SetFreq","Set Radio Freq to %llu  Bytes sent to radio (5 or 6 bytes) in BCD: %02X %02X %02X %02X %02X (%02X)\n", Freq, vfo_dec[0], vfo_dec[1], vfo_dec[2], vfo_dec[3], vfo_dec[4], vfo_dec[5]);
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
    //Serial.printf(" VFO: < 10G Bands = Reversed hex to DEC byte %02X %02X %02X %02X %02X %02X\n", vfo_dec[0], vfo_dec[1], vfo_dec[2], vfo_dec[3], vfo_dec[4], vfo_dec[5]);
  } else {
    len = 6;
    for (uint8_t i = 0; i < len; i++) {
      uint64_t x = vfo % 100;
      vfo_dec[i] = bcdByteEncode((uint8_t)(x));
      vfo = vfo / 100;
    }
    //Serial.printf(" VFO: > 10G Bands = Reversed hex to DEC byte %02X %02X %02Xpass_PC_to_radio %02X %02X %02X %02X\n", vfo_dec[0], vfo_dec[1], vfo_dec[2], vfo_dec[3], vfo_dec[4], vfo_dec[5], vfo_dec[6]);
  }
  return len;  // 5 or 6
}

uint8_t IC_905_Input_scan(void) 
{
    uint8_t pattern = 0;
        pattern |= gpio_get_level(GPIO_PTT_INPUT);       // PTT inut from Radio SEND jack  Gnd - TX
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
        case  BAND_3cm  : GPIO_Out(DECODE_BAND10G);  break;   //10.368.1G
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
    }
}

void GPIO_PTT_Out(uint8_t pattern, bool _PTT_state)
{   
    uint8_t PTT_state = _PTT_state ? 0xFF : 0;

    char bin_num[9] ={};
    printBinaryWithPadding(pattern, bin_num);
    ESP_LOGI("GPIO_PTT_Out", "PTT Output Binary %s   PTT Output Hex 0x%02X   PTT Level at CPU pin %d", bin_num, pattern, _PTT_state);
    gpio_set_level(GPIO_PTT_OUTPUT_144,  (pattern & 0x01 & PTT_state) ? 1 : 0);  // bit 0 HF/50
    gpio_set_level(GPIO_PTT_OUTPUT_430,  (pattern & 0x02 & PTT_state) ? 1 : 0);  // bit 1 144
    gpio_set_level(GPIO_PTT_OUTPUT_1200, (pattern & 0x04 & PTT_state) ? 1 : 0);  // bit 2 222
    gpio_set_level(GPIO_PTT_OUTPUT_2300, (pattern & 0x08 & PTT_state) ? 1 : 0);  // bit 3 432
    gpio_set_level(GPIO_PTT_OUTPUT_5600, (pattern & 0x10 & PTT_state) ? 1 : 0);  // bit 4 902/903
    gpio_set_level(GPIO_PTT_OUTPUT_10G,  (pattern & 0x20 & PTT_state) ? 1 : 0);  // bit 5 1296
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
                if (read_buffer[3] == radio_address) {
                    if (read_buffer[2] == CONTROLLER_ADDRESS || read_buffer[2] == BROADCAST_ADDRESS) {

                        for (cmd_num = CIV_C_F_SEND; cmd_num < End_of_Cmd_List; cmd_num++)  // loop through the command list structure looking for a pattern match
                        {
                            //ESP_LOGI("processCatMessages", "processCatMessageslist: list index = "); DPRINTLN(cmd_num);
                            for (i = 1; i <= cmd_List[cmd_num].cmdData[0]; i++)  // start at the highest and search down. Break out if no match. Make it to the bottom and you have a match
                            {
                                //ESP_LOGI("processCatMessages", "processCatMessages: byte index = "); DPRINTLN(i);
                                //Serial.printf("processCatMessages: cmd_num=%d from radio, current byte from radio = %X  next byte=%X, on remote length=%d and cmd=%X\n",cmd_num, read_buffer[3+i], read_buffer[3+i+1], cmd_List[cmd_num].cmdData[0], cmd_List[cmd_num].cmdData[1]);
                                if (cmd_List[cmd_num].cmdData[i] != read_buffer[3 + i]) {
                                //ESP_LOGI("processCatMessages", "processCatMessages: Skip this one - Matched 1 element: look at next field, if any left. CMD Body Length = ");
                                //ESP_LOGI("processCatMessages", cmd_List[cmd_num].cmdData[0]); DPRINTF(" CMD  = "); Serial.print(cmd_List[cmd_num].cmdData[i], HEX);DPRINTF(" next RX byte = "); DPRINTLN(read_buffer[3+i+1],HEX);
                                match = 0;
                                break;
                                }
                                match++;
                                //ESP_LOGI("processCatMessages", "processCatMessages: Possible Match: Len = "); Serial.print(cmd_List[cmd_num].cmdData[0],DEC); DPRINTF("  CMD1 = "); Serial.print(read_buffer[4],HEX);
                                //ESP_LOGI("processCatMessages", " CMD2  = "); Serial.print(read_buffer[5],HEX); DPRINTF(" Data1/Term  = "); DPRINTLN(read_buffer[6],HEX);
                            }

                            //if (read_buffer[3+i] == STOP_BYTE)  // if the next byte is not a stop byte then it is thge next cmd byte or maybe a data byte, depends on cmd length

                            if (match && (match == cmd_List[cmd_num].cmdData[0])) 
                            {
                                //ESP_LOGI("processCatMessages", "processCatMessages:    FOUND MATCH: Len = "); Serial.print(cmd_List[cmd_num].cmdData[0],DEC); DPRINTF("  CMD1 = "); Serial.print(read_buffer[4],HEX);
                                //ESP_LOGI("processCatMessages", " CMD2  = "); Serial.print(read_buffer[5],HEX);  DPRINTF(" Data1/Term  = "); Serial.print(read_buffer[6],HEX); DPRINTF("  Message Length = "); DPRINTLN(msg_len);
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
                            ESP_LOGI("processCatMessages", "processCatMessages: No message match found - cmd_num = %d  read_buffer[4 & 5] = %X %X\n", cmd_num, read_buffer[4], read_buffer[5]);
                            //knowncommand = false;
                        } 
                        else 
                        {
                            ESP_LOGI("processCatMessages", "Call CIV_Action\n");
                            CIV_Action(cmd_num, data_start_idx, data_len, msg_len, read_buffer);
                        }
                    }  // is controller address
                }    // is radio address
            }      // is preamble
        }        // readline
    }          // while
}
/*
void init_OLED(void)
{
    Serial.println(F("Start SSD1306 OLED display Init"));
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    delay(500);
    // Clear the buffer
    display.clearDisplay();
    // Draw a single pixel in white
    display.drawPixel(10, 10, SSD1306_WHITE);
    // Show the display buffer on the screen. You MUST call display() after
    // drawing commands to make them visible on screen!
    display.display();
}
*/
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
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
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
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // De-Glitch PTT
    gpio_glitch_filter_handle_t filter;

    //gpio_flex_glitch_filter_config_t filter_config = {GLITCH_FILTER_CLK_SRC_DEFAULT, GPIO_PTT_INPUT, 5000 , 2000};
    //ESP_ERROR_CHECK(gpio_new_flex_glitch_filter(&filter_config, &filter));
    
    gpio_pin_glitch_filter_config_t filter_config = {GLITCH_FILTER_CLK_SRC_DEFAULT, GPIO_PTT_INPUT};
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
    gpio_isr_handler_add(GPIO_PTT_INPUT, gpio_isr_handler, (void*) GPIO_PTT_INPUT);
}

/**
 * @brief Main application
 *
 * Here we open a USB CDC device and send some data to it
 */
void app_main(void)
{
    ESP_LOGI(TAG, "IC-905 USB Band Decoder and PTT Breakout - K7MDL Jan 2025");

    setup_IO();

    // create a new rtos task for main radio control loop -- not in use yet
    BaseType_t TX_task_created = xTaskCreate(usb_TX_task, "usb_TX", 4096, xTaskGetCurrentTaskHandle(), EXAMPLE_USB_HOST_PRIORITY, NULL);
    assert(TX_task_created == pdTRUE);

    // Setup our USB Host port
    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem);

    // Install USB Host driver. Should only be called once in entire application
    ESP_LOGI(TAG, "Installing USB Host");
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Create a task that will handle USB library events
    BaseType_t task_created = xTaskCreate(usb_lib_task, "usb_lib", 4096, xTaskGetCurrentTaskHandle(), EXAMPLE_USB_HOST_PRIORITY, NULL);
    assert(task_created == pdTRUE);

    ESP_LOGI(TAG, "Installing CDC-ACM driver");
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 1000,
        .out_buffer_size = 512,
        .in_buffer_size = 512,
        .user_arg = NULL,
        .event_cb = handle_event,
        .data_cb = handle_rx
    };

    while (true) {
        //cdc_acm_dev_hdl_t cdc_dev = NULL;

        // Open USB device from tusb_serial_device example example. Either single or dual port configuration.
        ESP_LOGI(TAG, "Opening CDC ACM device 0x%04X:0x%04X...", EXAMPLE_USB_DEVICE_VID, EXAMPLE_USB_DEVICE_PID);
        esp_err_t err = cdc_acm_host_open(EXAMPLE_USB_DEVICE_VID, EXAMPLE_USB_DEVICE_PID, 0, &dev_config, &cdc_dev);
        if (ESP_OK != err) {
            ESP_LOGI(TAG, "Opening CDC ACM device 0x%04X:0x%04X...", EXAMPLE_USB_DEVICE_DUAL_VID, EXAMPLE_USB_DEVICE_DUAL_PID);
            err = cdc_acm_host_open(EXAMPLE_USB_DEVICE_DUAL_VID, EXAMPLE_USB_DEVICE_DUAL_PID, 0, &dev_config, &cdc_dev);
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

        // We are done. Wait for device disconnection and start over
        //ESP_LOGI(TAG, "Example finished successfully! You can reconnect the device to run again.");

        USBH_connected = true;

        // We can now send to CI-V
        vTaskDelay(20);  // Time for radio to to beready for comms after connection
        
        ESP_LOGI(TAG, "***Starting CI-V communications");
        ESP_ERROR_CHECK(cdc_acm_host_set_control_line_state(cdc_dev, false, false));
        //SetFreq(145500000);  // Used for testing
       
        ESP_LOGI(TAG, "***Get frequency from radio");
        sendCatRequest(CIV_C_F_READ, 0, 0);  // Get current VFO     
        vTaskDelay(pdMS_TO_TICKS(2));

        // Get started by retrieving frequency, mode, time & location and time offset.
        ESP_LOGI(TAG, "***Get extended mode info from radio");
        sendCatRequest(CIV_C_F26A, 0, 0);  // Get extended info -  mode, filter, and datamode status
        vTaskDelay(pdMS_TO_TICKS(2));

        ESP_LOGI(TAG, "***Get UTC Time offset from radio");
        if (radio_address == IC905)                  //905
            sendCatRequest(CIV_C_UTC_READ_905, 0, 0);  //CMD_READ_FREQ);
        else if (radio_address == IC705)             // 705
            sendCatRequest(CIV_C_UTC_READ_705, 0, 0);  //CMD_READ_FREQ);
        vTaskDelay(pdMS_TO_TICKS(2));
        
        ESP_LOGI(TAG, "***Get time, date and position from radio.  We wil then calculate grid square");
        sendCatRequest(CIV_C_MY_POSIT_READ, 0, 0);  //CMD_READ_FREQ);
        vTaskDelay(pdMS_TO_TICKS(5));

        ESP_LOGI(TAG, "***Now wait for radio dial and band change messaages");
        // usb_TX_task() is where our program runs within now.  
        // handler_rx takes care of received events
        // Do not Transmit to USB (such as call send_CAT_Request()) from the handler_rx, they will overlap and cause timeouts.
        // Incoming CIV data appears in handler_rx whih in turn calls process messages and CIV_Action. 
        // If anything needs to be TX back to the radio by those processes like mode calling get ext mode, 
        // or read frequency() calling get ext mode, set a flag and let usb_TX_task() handle it.

        // program waits here while tasks run handling events
        xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);
    }
}
