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
//  Skipping gpio pins 8 and 9 for future i2c use
// *************************************************************************************************************

#define GPIO_PTT_INPUT          GPIO_NUM_1   // 

#ifdef PROTOTYPE
    // gpio pins 0, 3, 45, and 46 are CPU option "strapping pins" and should not be used
    // pins 8 and 9 reserved for future i2c bus usage
    #define GPIO_BAND_OUTPUT_144    GPIO_NUM_5    // default Voltage 0V
    #define GPIO_BAND_OUTPUT_430    GPIO_NUM_6    // default Voltage 0V
    #define GPIO_BAND_OUTPUT_1200   GPIO_NUM_7    // default Voltage 0V
    #define GPIO_BAND_OUTPUT_2300   GPIO_NUM_10   // default Voltage 0V
    #define GPIO_BAND_OUTPUT_5600   GPIO_NUM_11   // default Voltage 0V
    #define GPIO_BAND_OUTPUT_10G    GPIO_NUM_12   // default Voltage 0V

    #define GPIO_PTT_OUTPUT_144     GPIO_NUM_13   // default Voltage 0V
    #define GPIO_PTT_OUTPUT_430     GPIO_NUM_14   // default Voltage 0V
    #define GPIO_PTT_OUTPUT_1200    GPIO_NUM_15   // default Voltage 0V
    #define GPIO_PTT_OUTPUT_2300    GPIO_NUM_16   // default Voltage 0V
    #define GPIO_PTT_OUTPUT_5600    GPIO_NUM_17   // Also UART1 TX (not used)
    #define GPIO_PTT_OUTPUT_10G     GPIO_NUM_38   // Also UART1 RX (not used)  moved from 18 to make way for the ADC and free up 8/9 for i2c in future.
#else
    #define GPIO_BAND_OUTPUT_144    GPIO_NUM_5    // default Voltage 0V
    #define GPIO_BAND_OUTPUT_430    GPIO_NUM_6    // default Voltage 0V
    #define GPIO_BAND_OUTPUT_1200   GPIO_NUM_7    // default Voltage 0V
    #define GPIO_BAND_OUTPUT_2300   GPIO_NUM_15   // default Voltage 0V
    #define GPIO_BAND_OUTPUT_5600   GPIO_NUM_16   // default Voltage 0V
    #define GPIO_BAND_OUTPUT_10G    GPIO_NUM_17   // default Voltage 0V

    #define GPIO_PTT_OUTPUT_144     GPIO_NUM_18   // default Voltage 0V
    #define GPIO_PTT_OUTPUT_430     GPIO_NUM_10   // default Voltage 0V
    #define GPIO_PTT_OUTPUT_1200    GPIO_NUM_11   // default Voltage 0V
    #define GPIO_PTT_OUTPUT_2300    GPIO_NUM_12   // default Voltage 0V
    #define GPIO_PTT_OUTPUT_5600    GPIO_NUM_13   // Also UART1 TX (not used)
    #define GPIO_PTT_OUTPUT_10G     GPIO_NUM_14   // Also UART1 RX (not used)  moved from 18 to make way for the ADC and free up 8/9 for i2c in future.
#endif
// *************************************************************************************************************
//
//   Assign DECODER pattern to each band as needed
//
//**************************************************************************************************************

// BAND DECODE INPUT PATTERN MAPPING TO BAND
// Likey not used, TBD.  These are not pins, just a band index.
#define DECODE_INPUT_DUMMY        0    //Dummy Row
#define DECODE_INPUT_BAND144      1    //144
#define DECODE_INPUT_BAND430      2    //432
#define DECODE_INPUT_BAND1200     3    //1296
#define DECODE_INPUT_BAND2300     4    //2400
#define DECODE_INPUT_BAND5600     5    //5760M
#define DECODE_INPUT_BAND10G      6    //10G


#ifdef REMOTE_BOARD 
    // Band Decode Output pattern is 3 wire.  Account for inversion on USB side
    #define DECODE_BAND_DUMMY    (0x00)    //Dummy Row - same as Band 1.  Buffered output is 0xFF
    #define DECODE_BAND144       (0x00)    //144 -  Buffered output will be 0xFF for 111 at the remote board band 1
    #define DECODE_BAND430       (0x01)    //432 -  Buffered output will be 0xFE for 110 at the remote board band 2
    #define DECODE_BAND1200      (0x02)    //1296 - Buffered output will be 0xFD for 101 at the remote board band 3
    #define DECODE_BAND2300      (0x03)    //2400 - Buffered output will be 0xFC for 100 at the remote board band 4
    #define DECODE_BAND5600      (0x04)    //5760 - Buffered output will be 0xFB for 011 at the remote board band 5
    #define DECODE_BAND10G       (0x05)    //10G -  Buffered output will be 0xFA for 010 at the remote board band 6

    // inverted for buffer.  Set 4th port low at remote board for TX=ON, raise high for TX=OFF.  Use the same PTT port regardless of band
    #define DECODE_DUMMY_PTT     (0x00)    //Dummy Row
    #define DECODE_BAND144_PTT   (0x08)    //144_PTT - set high because of inversion in USB side to get low for TX at remote input
    #define DECODE_BAND430_PTT   (0x08)    //432_PTT
    #define DECODE_BAND1200_PTT  (0x08)    //1296_PTT
    #define DECODE_BAND2300_PTT  (0x08)    //2400_PTT
    #define DECODE_BAND5600_PTT  (0x08)    //5760_PTT
    #define DECODE_BAND10G_PTT   (0x08)    //10G_PTT
#else   // Normal 1 of 6 outputs pattern]
    // Band Decode Output pattern - default is  1 of 6 pattern.  Can make it anything you need.
    #define DECODE_BAND_DUMMY    (0x00)    //Dummy Row
    #define DECODE_BAND144       (0x01)    //144
    #define DECODE_BAND430       (0x02)    //432
    #define DECODE_BAND1200      (0x04)    //1296
    #define DECODE_BAND2300      (0x08)    //2400
    #define DECODE_BAND5600      (0x10)    //5760
    #define DECODE_BAND10G       (0x20)    //10G

    // inverted for buffer.  Set low for TX=OFF, raise high for TX=ON and make TX=GND on buffer output
    #define DECODE_DUMMY_PTT     (0x00)    //Dummy Row
    #define DECODE_BAND144_PTT   (0x01)    //144_PTT
    #define DECODE_BAND430_PTT   (0x02)    //432_PTT
    #define DECODE_BAND1200_PTT  (0x04)    //1296_PTT
    #define DECODE_BAND2300_PTT  (0x08)    //2400_PTT
    #define DECODE_BAND5600_PTT  (0x10)    //5760_PTT
    #define DECODE_BAND10G_PTT   (0x20)    //10G_PTT
#endif  // REMOTE_BOARD 

#endif  // header file Decoder.h