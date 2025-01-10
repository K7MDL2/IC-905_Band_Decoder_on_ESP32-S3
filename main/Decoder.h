/*
*   Decoder.h
*   Jan 2025 by K7MDL
*/

#ifndef DECODER_H_
#define DECODER_H_

#include <stdint.h>
//#include "IC905_ESP32-S3_PTT_Breakout.h"

void Band_Decode_Output(uint8_t band);
void GPIO_Out(uint8_t pattern);
void PTT_Output(uint8_t band, bool PTT_state);
void GPIO_PTT_Out(uint8_t pattern, bool PTT_state);

enum band_idx { DUMMY,
                BAND_2M,
                BAND_70cm,
                BAND_23cm,
                BAND_13cm,
                BAND_6cm,
                BAND_3cm,
                NUM_OF_BANDS };

#define GPIO_PIN_NOT_USED   255  // use for any pin not in use below

// *************************************************************************************************************
// Make IO Pin assignments here. 
//  These are IO pins on the CPU module intended for Band decode input and PTT input and IF switch control
//  PTT is the only IO pin that requires high speed scanning.  
// *************************************************************************************************************

#define GPIO_PTT_INPUT          GPIO_NUM_1   // CPU pin 39, default Voltage 0V

#define GPIO_BAND_OUTPUT_144    GPIO_NUM_5   // CPU pin 5, default Voltage 0V
#define GPIO_BAND_OUTPUT_430    GPIO_NUM_6   // CPU pin 6, default Voltage 0V
#define GPIO_BAND_OUTPUT_1200   GPIO_NUM_7   // CPU pin 7, default Voltage 0V
#define GPIO_BAND_OUTPUT_2300   GPIO_NUM_10   // CPU pin 18, default Voltage 0V
#define GPIO_BAND_OUTPUT_5600   GPIO_NUM_11   // CPU pin 19, default Voltage 0V
#define GPIO_BAND_OUTPUT_10G    GPIO_NUM_12   // CPU pin 20, default Voltage 0V

#define GPIO_PTT_OUTPUT_144     GPIO_NUM_13   // CPU pin 21, default Voltage 0V
#define GPIO_PTT_OUTPUT_430     GPIO_NUM_14   // CPU pin 22, default Voltage 0V
#define GPIO_PTT_OUTPUT_1200    GPIO_NUM_15   // CPU pin 8, default Voltage 0V
#define GPIO_PTT_OUTPUT_2300    GPIO_NUM_16   // CPU pin 9, default Voltage 0V
#define GPIO_PTT_OUTPUT_5600    GPIO_NUM_17   // CPU pin 10, default Voltage 0V
#define GPIO_PTT_OUTPUT_10G     GPIO_NUM_18   // CPU pin 11, default Voltage 0V

// *************************************************************************************************************
//
//   Assign DECODER pattern to each band as needed
//
//**************************************************************************************************************

// BAND DECODE INPUT PATTERN MAPPING TO BAND
// Likey not used, TBD  These are not pins, just a band index.
#define DECODE_INPUT_DUMMY        0    //Dummy Row
#define DECODE_INPUT_BAND144      1    //2M
#define DECODE_INPUT_BAND430      2    //432
#define DECODE_INPUT_BAND1200     3    //1296
#define DECODE_INPUT_BAND2300     4    //2400
#define DECODE_INPUT_BAND5600     5    //5760M
#define DECODE_INPUT_BAND10G      6    //10.368.1G


// Band Decode Output pattern
// Example: For Core module with just the 4In/8out module, there are 8 output.  Use lowest half of the value.
#define DECODE_BAND_DUMMY   (0x00)    //Dummy Row
#define DECODE_BAND144      (0x01)    //2M
#define DECODE_BAND430      (0x02)    //432
#define DECODE_BAND1200     (0x04)    //1296
#define DECODE_BAND2300     (0x08)    //2400
#define DECODE_BAND5600     (0x10)    //5760M
#define DECODE_BAND10G      (0x20)    //10.368.1G

// invert for buffer.  Set low for TX=OFF, raise high for TX=ON and make TX=GND on buffer output
#define DECODE_DUMMY_PTT     (0x00)    //Dummy Row
#define DECODE_BAND144_PTT   (0x01)    //2M_PTT
#define DECODE_BAND430_PTT   (0x02)    //432_PTT
#define DECODE_BAND1200_PTT  (0x04)    //1296_PTT
#define DECODE_BAND2300_PTT  (0x08)    //2400_PTT
#define DECODE_BAND5600_PTT  (0x10)    //5760_PTT
#define DECODE_BAND10G_PTT   (0x20)    //10.368.1G_PTT

#endif