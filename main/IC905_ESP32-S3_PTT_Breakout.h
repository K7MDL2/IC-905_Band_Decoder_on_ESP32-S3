/*
* IC905_ESP32-S3_PTT_Breakout.h
*/

#include <inttypes.h>
#include "CIV.h"

#define IC705 0xA4
#define IC905 0xAC
#define RADIO_ADDR IC905
#define WIRED_PTT   1           // 1 = use the wired input for fastest PTT response time
                                // 0 = poll radio for TX status. Polling delay can be adjusted with parameters below.
// NOTE: With a single USB virtual Serial port to the PC, ANY debug on Serial will interfere with a program like WSJT-X passing through to teh radio.
#define USBHOST                 // if no BLE or BTCLASSIC this must be enabled.   *** USB Host is not stable so far ****

#define PTT_DELAY    30       // PTT sequencing delay.  At start of PTT -> TX event, turns OFF IF switch to prevent
                              //   RF flowing while downstream relays are switching.

#define POLL_PTT_DEFAULT 247  // poll the radio for PTT status odd numbers to stagger them a bit
                              // USB on both the 705 and 905 respond to PTT requests slower on USB than BT on the 705.
                              // Also polls the wired inputs  Can go down to 25-45.  When using wired PTT set this slow.
#define POLL_PTT_USBHOST 262  // Dynamically changes value based on detected radio address.
                              // By observation, on USB, the radio only responds once every few seconds when the radio 
                              //   has not changed states.  It will immediately reply to a poll if the Tx state changed. 
                              //   Still have to poll fast for controlling external PTT, most requests will not be answered. 
                              //   Unlike other modes.  BT seems to answer every request. USB2 engine is likely the same in 
                              //   all radios, where BT got a capacity upgrade.  The 905 acts the same as the 905 (905 is USB only) 
                              //   Have not compared to a LAN connection.
#define POLL_RADIO_FREQ   708 // poll the radio for frequency
#define POLL_RADIO_UTC    998 // poll radio for time and location
#define POLL_RADIO_MODE  6101 // poll radio for extended mode, filter and datamode
#define POLL_RADIO_AGC   3403 // poll radio for AGC
#define POLL_RADIO_ATTN  3305 // poll radio for atten status
#define POLL_RADIO_PRE   3204 // poll radio for preamp status
#define POLL_RADIO_SPLIT 3102 // poll radio for split status

#define CONTROLLER_ADDRESS 0xE0  //Controller address
#define BROADCAST_ADDRESS 0x00
#define START_BYTE 0xFE  // Start byte
#define STOP_BYTE 0xFD   // Stop byte
#define CMD_READ_FREQ 0x03    // Read operating frequency data

//#define PC_PASSTHROUGH  // fwd through BT or USBHOST data to a PC if connected.  All debug must be off!
#ifndef PC_PASSTHROUGH        // shut off by default when PASSTHRU MODE is on
  #define PRINT_VFO_TO_SERIAL // uncomment to visually see VFO updates from the radio on Serial
  #define PRINT_PTT_TO_SERIAL // uncomment to visually see PTT updates from the radio on Serial
  #define NO_SEND  // block changes to radio from controller - used for PC pass thru
#else
  #undef NO_SEND  // block changes to radio from controller - used for PC pass thru
#endif

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define EXAMPLE_USB_HOST_PRIORITY   (20)

// VID and PID for Icom IC-905 radio - has 2 serial channels and audio codec channel
// cannot open this device as acm.  Maybe it is the spectrum channel
// VID and PID of Icom 2nd device  // This is the CI-V bus serial data channel that we want.
// There are 2 serial (acm) channels. 2nd is typically GPS NMEA sdata strings from the radio.
#define EXAMPLE_USB_DEVICE_VID          (0xc26)
#define EXAMPLE_USB_DEVICE_PID          (0x43) // 2nd device on IC-905
//#define EXAMPLE_USB_DEVICE_DUAL_VID     (0x451)
//#define EXAMPLE_USB_DEVICE_DUAL_PID     (0x2046) // 1st device on IC-905

// 3rd icom device is audio codec

// VID and PID for ESP32-S3 dev kit modules running acm device firmware
//#define EXAMPLE_USB_DEVICE_VID      (0x303A)
//#define EXAMPLE_USB_DEVICE_PID      (0x4001) // 0x303A:0x4001 (TinyUSB CDC device)
#define EXAMPLE_USB_DEVICE_DUAL_VID (0x303A)
#define EXAMPLE_USB_DEVICE_DUAL_PID (0x4001) // 0x303A:0x4002 (TinyUSB Dual CDC device)

#define EXAMPLE_TX_STRING           ("IC905 PTT Breakout Test String")
#define EXAMPLE_TX_TIMEOUT_MS       (1000)

//#define SEE_RAW_RX // see raw hex messages from radio
//#define SEE_RAW_TX // see raw hex messages from radio

struct Bands {
  char band_name[6];    // Freindly name or label.  Default here but can be changed by user.
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
