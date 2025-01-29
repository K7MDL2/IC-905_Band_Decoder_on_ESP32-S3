/*
*   IC905_ESP32-S3_PTT_Breakout.h
*   Jan 2025 by K7MDL
*/

#ifndef _IC905_H_
#define _IC905_H_

#include <inttypes.h>
#include "CIV.h"

// --------------------------------------------------------------------------------------------------------
//  Start of User selected values
// --------------------------------------------------------------------------------------------------------
#define ALL_RADIOS 0
#define IC705 0xA4
#define IC905 0xAC
#define IC9700 0xA2
#define RADIO_ADDR ALL_RADIOS   // default ALL_RADIOS

#define LED_BRIGHT_LEVEL  600   // (0-8100)  Sets all Discrete LEDs brightness level. Default=600

//#define GET_EXT_MODE_INFO     // Enable extended mode info poll after normal mode result received.  
                                // Only used when there is a display and may interfere with WSJT-X if PC Passthrough on

//#define POLL_FOR_TIME         // Polls every 1 second for time and location info from radio
                                // Only used when there is a display and may interfere with WSJT-X if PC Passthrough on

#define WIRED_PTT   1           // 1 = use the wired input for fastest PTT response time
                                // 0 = poll radio for TX status. Polling delay can be adjusted with parameters below.

#define PTT_DELAY    30       // PTT sequencing delay. Not used yet. 
                              // At start of PTT -> TX event, turns OFF IF switch to prevent
                              // RF flowing while downstream relays are switching.

#define POLL_PTT_DEFAULT 247  // poll the radio for PTT status odd numbers to stagger them a bit
                              // Also polls the wired inputs  Can go down to 25-45.  When using wired PTT set this slow.

#define POLL_RADIO_UTC    998 // poll radio for time and location

#define POLL_RADIO_MODE  6101 // poll radio for extended mode, filter and datamode

#define USE_LEDS  // operate band/PTT status LEDS (likely instead of using a OLED).

//#define RGB_LED_PIN 48      // RGB LED on DevKitC-1 is GPIO48

//#define ATOMS3                // Compile with setting for the M5Stack M5AtomS3 which features a color LCD

//#define PROTOTYPE           // PIN and ADC assignments unique to the prototype build

//#define CIV_SERIAL            // Pass through CI_V bus data on serial (USB or analog) between radio and PC

//#define UART_DEBUG            // 3rd UART channel for debugging radio CI-V to Radio bridging


// --------------------------------------------------------------------------------------------------------
//   End of user selected values
// --------------------------------------------------------------------------------------------------------

#ifdef ATOMS3
    #define POLL_FOR_TIME
    #undef USE_LEDS
#endif

#define CONTROLLER_ADDRESS 0xE0   // E1 used by wfView  //Controller address - not used for receive validation, used on TX to radio.
#define BROADCAST_ADDRESS 0x00
#define START_BYTE 0xFE  // Start byte
#define STOP_BYTE 0xFD   // Stop byte
#define CMD_READ_FREQ 0x03    // Read operating frequency data

#ifndef CIV_SERIAL        // shut off by default when PASSTHRU MODE is on
  #define PRINT_VFO_TO_SERIAL // uncomment to visually see VFO updates from the radio on Serial
  #define PRINT_PTT_TO_SERIAL // uncomment to visually see PTT updates from the radio on Serial
  #define NO_SEND  // block changes to radio from controller - used for PC pass thru
#else
  #undef NO_SEND  // block changes to radio from controller - used for PC pass thru
#endif

#define USB_HOST_PRIORITY   (20)

// VID and PID for Icom IC-905 radio - has 2 serial channels and audio codec channel
// cannot open this device as acm.  Maybe it is the spectrum channel
// VID and PID of Icom 2nd device  // This is the CI-V bus serial data channel that we want.
// There are 2 serial (acm) channels. 2nd is typically GPS NMEA sdata strings from the radio.
// 3rd icom device is audio codec/  TEH ESP can only handle 8 pipes, 4 per serial, channel (one is EP0)
#if (RADIO_ADDR == IC905)
  #define USB_DEVICE_VID          (0x0c26)
  #define USB_DEVICE_PID          (0x0043) // IC-905
#elif (RADIO_ADDR == IC705)
  #define USB_DEVICE_VID          (0x0c26)
  #define USB_DEVICE_PID          (0x0036) // IC-705
#elif (RADIO_ADDR == IC9700)
  #define USB_DEVICE_VID          (0x10C4)
  #define USB_DEVICE_PID          (0xEA60) // IC-9700
#endif
  
// Will also look for a 2nd port in case of switching to a dev board.
// VID and PID for ESP32-S3 dev kit modules running acm device firmware
//#define EXAMPLE_USB_DEVICE_VID      (0x303A)
//#define EXAMPLE_USB_DEVICE_PID      (0x4001) // 0x303A:0x4001 (TinyUSB CDC device)
#define USB_DEVICE_DUAL_VID (0x303A)
#define USB_DEVICE_DUAL_PID (0x4001) // 0x303A:0x4002 (TinyUSB Dual CDC device)

#define TX_STRING           ("IC905 PTT Breakout Test String")
#define TX_TIMEOUT_MS       (1000)

//#define SEE_RAW_RX // see raw hex messages from radio
//#define SEE_RAW_TX // see raw hex messages from radio

#ifdef PROTOTYPE
  // For ADC to read the LED brightness pot
  #define ADC2_CHAN0          ADC_CHANNEL_7
  #define ADC_ATTEN           ADC_ATTEN_DB_12
#else
  // For ADC to read the LED brightness pot
  #define ADC1_CHAN0          ADC_CHANNEL_3
  #define ADC_ATTEN           ADC_ATTEN_DB_12
#endif

#define BLINK_GPIO GPIO_NUM_48
//#define CONFIG_BLINK_LED_STRIP

struct Bands {
  char band_name[6];    // Friendly name or label.  Default here but can be changed by user.
  uint64_t edge_lower;  // band edge limits for TX and for when to change to next band when tuning up or down.
  uint64_t edge_upper;
  uint64_t Xvtr_offset;  // Offset to add to radio frequency.
                         // When all is correct, it will be within the band limits and allow PTT and Band decoder outputs
  uint64_t VFO_last;     // store the last used frequency on each band.
                         // for XVTR bands subtract the LO offset and send the result to the radio
  uint8_t mode_idx;      // current mode stored as indexc to the modelist table.
  uint8_t filt;          // current fiult soreds in teh modelist table
  uint8_t datamode;
  uint8_t agc;            // store last agc.  Some radio/band/mode combos only have 1.
  uint8_t preamp;         // some bands there is no preamp (2.4G+ on 905).  Some radios/bands/modes combos have 1 preamp level, others have 2 levels.
  uint8_t atten;          // some bands there is no atten (some on 905).  Some radios/bands/mode combos have 1 atten level, others have more. 
  uint8_t split;          // Split mode on or off
  uint8_t InputMap;       // If input pattern matches this value, then select this band.  First match wins.
};

#define M5ATOMS3 11

#ifdef ATOMS3
//#if defined ( CONFIG_IDF_TARGET_ESP32S3 )
    #ifdef __M5GFX_M5ATOMDISPLAY__
      #include <M5Unified.h>  // kills off USB Host
      //#include <M5AtomS3.h>
      #define ATOMS3
    #else 
      //#include <M5CoreS3.h>   // Mov 2024 latest M5Unified now supports M5CoreS3
      #include <M5Unified.h>  // kills off USB Host
    #endif
#endif  // CONFIG_IDF_TARGET_ESP32S3

#endif // _IC905_H_