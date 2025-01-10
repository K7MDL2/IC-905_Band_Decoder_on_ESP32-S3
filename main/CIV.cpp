/*  
*   CIV.cpp
*   Jan 2025 by K7MDL
*/

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "math.h"
#include "IC905_ESP32-S3_PTT_Breakout.h"
//#include "TimeLib.h"
#include "time.h"
#include <sys/time.h>
#include <ctime>
#include "CIV.h"

static const char *TAG = "CIV";

// CIV related stuff
//void CIV_Action(const uint8_t cmd_num, const uint8_t data_start_idx, const uint8_t data_len, const uint8_t msg_len, const uint8_t *rd_buffer);
extern struct Bands bands[]; 
char Grid_Square[GRIDSQUARE_LEN];   /* 10 char grid square max to be used - store d here as last known good Grid Flash icon if no valid GPS input*/
extern uint8_t UTC;    // 0 local time, 1 UTC time
extern void read_Frequency(uint64_t freq, uint8_t data_len);
extern bool PTT;
static bool TX_last = 0;
int hr_off;  // time offsets to apply to UTC time
int min_off;
int shift_dir;  // + or -
//tmElements_t tm;
time_t prevDisplay = 0; // When the digital clock was displayed
extern bool use_wired_PTT;
extern uint8_t band;
//extern void PTT_Output(uint8_t band, bool PTT_state);
extern uint8_t radio_address;
struct position p[1] = {};
extern uint64_t frequency;
extern void sendCatRequest(const uint8_t cmd_num, const uint8_t Data[], const uint8_t Data_len);  // first byte in Data is length
extern bool get_ext_mode_flag;
extern struct tm tm;
extern time_t t;

uint8_t radio_mode;      // mode from radio messages
uint8_t radio_filter;    // filter from radio messages
uint8_t radio_data;      // filter from radio messages
int16_t radio_RIT;    	// RIT offest 9.999KHz to -9.999KHz from radio CI-V
int32_t radio_DUP;		// Radio Duplex Offset
uint8_t radio_RIT_On_Off;
uint8_t radio_XIT_On_Off;

struct cmdList cmd_List[End_of_Cmd_List] = {
    {CIV_C_F_SEND,          {1,0x00}},                      // send operating frequency to all
    {CIV_C_F1_SEND,         {1,0x05}},                      // send operating frequency to one
    {CIV_C_F_READ,          {1,0x03}},                      // read operating frequency
    {CIV_C_F26,        		  {1,0x26}},                      // read selected VFO m data, filt -  26 datafield template; selected VFO; mode, data on/off(0-1), filter (1-3);
    {CIV_C_F26A,        	  {2,0x26,0x00}},                 // read/set selected VFO m data, filt
    {CIV_C_F26B,       		  {2,0x26,0x01}},                 // read/set  un- selected VFO m data, filt
    {CIV_C_F25A_SEND,       {2,0x25,0x00}},                 // set selected VFO frequency
    {CIV_C_F25B_SEND,       {2,0x25,0x01}},                 // set un-selected VFO frequency

    {CIV_C_MOD_READ,        {1,0x04}},               	      // read Modulation Mode in use
    {CIV_C_MOD_SET,         {3,0x06,0x23,0x02}},  		      // set mode to ATV and FIL2, same 2 byte filed for cmds 1, 4, and 6
    {CIV_C_MOD_SEND ,       {1,0x01}},                      // send Modulation Mode to all
    {CIV_C_MOD1_SEND,       {1,0x06}},                      // send Modulation Mode to one
    {CIV_C_MOD_USB_F1_SEND, {3,0x06,0x01,0x01}},            // send USB Filter 1 
    {CIV_C_MOD_USB_SEND,    {2,0x06,0x01}},                 // send USB Filter 1 
    
    {CIV_C_USB_D0_F2_SEND,  {5,0x26,0x00,0x01,0x00,0x02}},  // selected VFO; mod USB; Data OFF; RX_filter F2;
    {CIV_C_USB_D1_F2_SEND,  {5,0x26,0x00,0x01,0x01,0x02}},  // selected VFO; mod USB; Data ON;  RX_filter F2;
    {CIV_C_LSB_D0_F2_SEND,  {5,0x26,0x00,0x00,0x00,0x02}},  // selected VFO; mod USB; Data OFF; RX_filter F2;
    {CIV_C_LSB_D1_F2_SEND,  {5,0x26,0x00,0x00,0x01,0x02}},  // selected VFO; mod USB; Data ON;  RX_filter F2;
    {CIV_C_FM_D1_F1_SEND,   {5,0x26,0x00,0x05,0x01,0x01}},  // selected VFO; mod USB; Data ON;  RX_filter F2;
    
    {CIV_C_ATTN_READ,   	  {1,0x11}},                  	  // Attn read state
    {CIV_C_ATTN_OFF,   		  {2,0x11,0x00}},                 // Attn OFF
    {CIV_C_ATTN_ON,    		  {2,0x11,0x10}},                 // Attn 10dB (144, 432, 1200 bands only)
    {CIV_C_SPLIT_READ,      {1,0x0F}},                      // read Split OFF
    {CIV_C_SPLIT_OFF_SEND,  {2,0x0F,0x00}},                 // set split OFF
    {CIV_C_SPLIT_ON_SEND,   {2,0x0F,0x01}},                 // Set split ON
    {CIV_C_RFGAIN,          {2,0x14,0x02}},                 // send/read RF Gain
    {CIV_C_AFGAIN,          {2,0x14,0x01}},                 // send/read AF Gain
    {CIV_C_RFPOWER,         {2,0x14,0x0A}},                 // send/read selected bands RF power
    {CIV_C_S_MTR_LVL,       {2,0x15,0x02}},                 // send/read S-meter level (00 00 to 02 55)  00 00 = S0, 01 20 = S9, 02 41 = S9+60dB
    {CIV_C_PREAMP_READ,     {2,0x16,0x02}},             	  // read preamp state
    {CIV_C_PREAMP_OFF,      {3,0x16,0x02,0x00}},            // send/read preamp 3rd byte is on or of for sending - 00 = OFF, 01 = ON
    {CIV_C_PREAMP_ON,       {3,0x16,0x02,0x00}},            // send/read preamp 3rd byte is on or of for sending - 00 = OFF, 01 = ON
    {CIV_C_PREAMP_ON2,      {3,0x16,0x02,0x02}},            // send/read preamp 3rd byte is on or of for sending - 00 = OFF, 01 = ON - not on 905
    {CIV_C_AGC_READ,        {2,0x16,0x12}},                 // send/read AGC  01 = FAST, 02 = MID, 03 = SLOW
    {CIV_C_AGC_FAST,        {3,0x16,0x12,0x01}},            // send/read AGC  01 = FAST, 02 = MID, 03 = SLOW
    {CIV_C_AGC_MID,         {3,0x16,0x12,0x02}},            // send/read AGC  01 = FAST, 02 = MID, 03 = SLOW
    {CIV_C_AGC_SLOW,        {3,0x16,0x12,0x03}},            // send/read AGC  01 = FAST, 02 = MID, 03 = SLOW
    {CIV_C_CW_MSGS,         {1,0x17}},                      // Send CW messages see page 17 of prog manual for char table
    {CIV_C_BSTACK,          {2,0x1A,0x01}},                 // send/read BandStack contents - see page 19 of prog manual.  
                                                                    // data byte 1 0xyy = Freq band code
                                                                    // dat abyte 2 0xzz = register code 01, 02 or 03
                                                                    // to read 432 band stack register 1 use 0x1A,0x01,0x02,0x01
    {CIV_C_MY_POSIT_READ,   {2,0x23,0x00}},          	    // read my GPS Position
    {CIV_C_MY_POSIT_DATA,   {1,0x23}},          	    	  // read my GPS Position
    {CIV_C_RF_POW,          {2,0x14,0x0A}},            		// send / read max RF power setting (0..255 == 0 .. 100%)
    {CIV_C_TRX_ON_OFF,      {1,0x18}},                 		// switch radio ON/OFF
    {CIV_C_TRX_ID,          {2,0x19,0x00}},            		// ID query
    {CIV_C_TX,              {2,0x1C,0x00}},            		// query of TX-State 00=OFF, 01=ON
    // the following three commands don't fit for IC7100 !!!
    {CIV_C_DATE,            {4,0x1A,0x05,0x00,0x94}},  		// + 0x20 0x20 0x04 0x27 for 27.4.2020
    {CIV_C_TIME,            {4,0x1A,0x05,0x00,0x95}},  		// + 0x19 0x57 for 19:57
    //{CIV_C_UTC,             {4,0x1A,0x05,0x00,0x96}},  		// + 0x01,0x00,0x00 = +1h delta of UTC to MEZ
    {CIV_C_UTC_READ_905,    {4,0x1A,0x05,0x01,0x81}},     //  Get UTC Offset
    //{CIV_C_UTC_SEND,        {4,0x1A,0x05,0x00,0x96}},  		// + 0x01,0x00,0x00 = +1h delta of UTC to MEZ
    {CIV_C_UTC_READ_705,    {4,0x1A,0x05,0x01,0x70}},  		// + 0x01,0x00,0x00 = +1h delta of UTC to MEZ
    {CIV_C_DUPLEX_READ,		  {1,0x0C}},          	    	  // read Duplex Offset  - has 3 bytes frequency offset data
    {CIV_C_DUPLEX_SEND,		  {1,0x0D}},	          	    	// send Duplex Offset
    {CIV_C_RIT_XIT,			    {2,0x21,0x00}},          	    // read or send RIT/XIT Offset  - has 3 bytes frequency offset data  XIT and RIT share this Offset value
    {CIV_C_RIT_ON_OFF,		  {2,0x21,0x01}},	          	  // send or send RIT ON or Off status 00 = , 01 = t
    {CIV_C_XIT_ON_OFF,		  {2,0x21,0x02}},	          	  // send or send XIT Offset
    {CIV_C_RADIO_OFF,		    {2,0x18,0x00}},	          	  // Turn Off the radio
    {CIV_C_RADIO_ON,		    {2,0x18,0x01}}	          	  // Turn on the radio
};

//
// Icom mode number, model Label, range of filt choices allowed (1, 2, or 3), and data mode allowed table (set for data capable modes only)
// per mode, can use filtX and datamode clumns to determine allowable value to write to the radio, or to allow for display.
// not all filters are avaiable for all odes.  FOr  example DD and WFM are FILT1 only. 
// To use filt column, test desired or received value against highest value in the field.  
// For datamode, simple check for 1 or 0. If o hen data mode must be off.  If 1, data mode must be on.  
// in a table search, that would change the displayed mode label from USB to USB-D for example, they are both the same base mode, 0x01.
// 

struct Modes_List modeList[MODES_NUM] = {
    {0x00, "LSB   ", 3, 0},
    {0x01, "USB   ", 3, 0},
    {0x02, "AM    ", 3, 0},
    {0x03, "CW    ", 3, 0},
    {0x04, "RTTY  ", 3, 0},
    {0x05, "FM    ", 3, 0},
    {0x06, "W-FM  ", 1, 0},  // on 705, NA for IC-905
    {0x07, "CW-R  ", 3, 0},
    {0x08, "RTTY-R", 3, 0},
    {0x17, "DV    ", 3, 0},  // hex 17 is 23 dec
    {0x00, "LSB-D ", 3, 1},
    {0x01, "USB-D ", 3, 1},
    {0x02, "AM-D  ", 3, 1},
    {0x05, "FM-D  ", 3, 1},
    {0x22, "DD    ", 1, 0},  // hex 22 is 34 dec  // not on 705, on 905
    {0x23, "ATV   ", 1, 0}   // hex 23 is 35 dec  // not on 705, on 905
 };

// states of radio's DC-Power (on/Off State)
const char radioOnOffStr[6][13] = {
  "RADIO_OFF",
  "RADIO_ON",
  "RADIO_OFF_TR",     // transit from OFF to ON
  "RADIO_ON_TR",      // transit from ON to OFF
  "RADIO_NDEF",       // don't know
  "RADIO_TOGGLE"
};

//---------------------------------------------------------------------------------------------------------

/*
unsigned int hexToDec(String hexString) {
    hexString.reserve(2);
    unsigned int decValue = 0;
    unsigned int nextInt;
    for (unsigned int i = 0; i < hexString.length(); i++) {
        nextInt = int(hexString.charAt(i));
        if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
        if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
        if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
        nextInt = constrain((signed)nextInt, 0, 15);
        decValue = (decValue * 16) + nextInt;
    }
    return decValue;
}
*/

// reverses a string 'str' of length 'len' 
void reverse(char *str, int len) 
{ 
    int i=0, j=len-1, temp; 
    while (i<j) 
    { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; j--; 
    } 
} 
 // Converts a given integer x to string str[].  d is the number 
 // of digits required in output. If d is more than the number 
 // of digits in x, then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while (x) 
    { 
        str[i++] = (x%10) + '0'; 
        x = x/10; 
    } 
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 

 // Converts a floating point number to string. 
void ftoa(float n, char *res, int afterpoint) 
{ 
    // Extract integer part 
    int ipart = (int)n; 
  
    // Extract floating part 
    float fpart = n - (float)ipart; 
  
    // convert integer part to string 
    int i = intToStr(ipart, res, 0); 
  
    // check for display option after point 
    if (afterpoint != 0) 
    { 
        res[i] = '.';  // add dot 
  
        // Get the value of fraction part upto given no. 
        // of points after dot. The third parameter is needed 
        // to handle cases like 233.007 
        fpart = fpart * pow(10, afterpoint); 
  
        intToStr((int)fpart, res + i + 1, afterpoint); 
    } 
}

/*
* The algorithm is fairly straightforward. The scaling array provides divisors to divide up the space into the required number of sections,
* which is 18 for for the field, 10 for the square, 24 for the subsquare, 10 for the extended square, then 24, then 10. 
* The limit is 6 pairs which is 2 more than is used even in the most detailed versions (8 characters in all). 
* The divisor is also used in the fmod function to narrow down the range to the next highest order square that we’re dividing up.
* The scaling array could be precalculated, but I figure the optimizing compiler would take care of that. 
* I also thought the values could be scaled up at the beginning of the function, then use integer arithmetic to do the conversion. 
* It might be a bit faster.
*
*/

/*
To run it “./geo lat long”, e.g. “./geo 43.999 -79.495” which yields FN03gx09
*/

/*  Parse the GPS NMEA ASCII GPGGA string for the time, latitude and longitude  */

//---------------------------------------------------------------------------------------------------------

int positionToMaidenhead(char m[])
{

    const int pairs=4;
    const double scaling[]={360.,360./18.,(360./18.)/10., \
    ((360./18.)/10.)/24.,(((360./18.)/10.)/24.)/10., \
    ((((360./18.)/10.)/24.)/10.)/24., \
    (((((360./18.)/10.)/24.)/10.)/24.)/10.};
    int i;
    int index;

    for (i=0;i<pairs;i++)
    {
        index = (int)floor(fmod((180.0+p->longitude), scaling[i])/scaling[i+1]); 
        m[i*2] = (i&1) ? 0x30+index : (i&2) ? 0x61+index : 0x41+index;
        index = (int)floor(fmod((90.0+p->latitude), (scaling[i]/2))/(scaling[i+1]/2));
        m[i*2+1] = (i&1) ? 0x30+index : (i&2) ? 0x61+index : 0x41+index;
    }
    m[pairs*2]=0;
    return 1;  // success
}

/*  Convert Alt and Lon to MH now */

int Convert_to_MH(void)
{
    char m[9];

   // if(GPS_Status == GPS_STATUS_LOCK_INVALID || msg_Complete == 0)   
   //     return 1;  /* if we are here with invalid data then exit.  LAt and LOnwill have text which cannot be computered of cour     */
    /*  Get from GPS Serial input later */
    p->latitude=atof(Latitude);
    p->longitude=atof(Longitude);

    if (positionToMaidenhead(m))
    {   
        strncpy(Grid_Square,m,8);
        return 1;  // Success               
    }
    else
    {
        strcpy(Grid_Square,"Invalid\0");
        return 0; // fail      /*  Can use later to skip displaying anything when have invalid or no GPS input   */
    }
}

void ConvertToMinutes(char _gps_msg[])
{
    double  longitude_d, longitude_m, latitude_d, latitude_m;
    char    ns='?', ew='?';
    char    Degrees[13];
    char    Minutes[13];
    char    tempstring[12];
    char *ptr = _gps_msg;

    strncpy(Latitude, ptr, 9);      
    strncpy(Degrees,Latitude,2);
    Degrees[2] = '\0';
    latitude_d = atoi(Degrees);
    strncpy(Minutes,&Latitude[2],7);
    Minutes[9] = '\0';
    latitude_m = atof(Minutes);
    latitude_m /= 60;
    latitude_d = latitude_d + latitude_m;    
    ftoa(latitude_d, tempstring, 6);  
  
    /*  use N/S to set neg or pos */ 
    ptr = strchr(ptr+1, ',');
    ns = ptr[1] == ',' ? '?' : ptr[1];  /*  get the north south letter */
    if(ns == 'S')
        sniprintf(Latitude, 11, "%c%s", '-', tempstring);       /*  the ftoa function does not handle neg numbers */
    else
        sniprintf(Latitude, 11, "%c%s", ' ', tempstring);    
  
    ptr = strchr(ptr+1, ',');
    strncpy(Longitude, ptr+1, 10);
    strncpy(Degrees,Longitude,3);
    Degrees[3] = '\0';
    longitude_d = atoi(Degrees);
    strncpy(Minutes,&Longitude[3],8);
    Minutes[9] = '\0';
    longitude_m = atof(Minutes);
    longitude_m /= 60;
    longitude_d = longitude_d + longitude_m;
    ftoa(longitude_d, tempstring, 9);
    
    ptr = strchr(ptr+1, ',');
    ew = ptr[1] == ',' ? '?' : ptr[1];
    
    /*  use E/W to set neg or pos */
    if(ew == 'W')
        sniprintf(Longitude, 13, "%c%s", '-', tempstring);        /*  the ftoa function does not handle neg numbers */    
    else
        sniprintf(Longitude, 13, "%c%s", ' ', tempstring); 
}        
 
// send to radio Mode, Data mode, and Filter
void SetMode(uint8_t  _band) {
  uint8_t data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};  // data template - first byte will stay 0 for selected VFO
  data[1] = modeList[bands[_band].mode_idx].mode_num;   // convert from our mode/datamode combo list index to separate values
  data[2] = bands[_band].datamode;
  data[3] = bands[_band].filt;
  sendCatRequest(CIV_C_F26, data, 4);
}

// send to radio Mode, Data mode, and Filter
void SetAGC(uint8_t  _band) {
  switch (bands[_band].agc) {
    case 1: sendCatRequest(CIV_C_AGC_FAST, 0, 0); break;
    case 2: sendCatRequest(CIV_C_AGC_MID, 0, 0); break;
    case 3:
    default: sendCatRequest(CIV_C_AGC_SLOW, 0, 0);  break;
  };
}

// set attenuator on radio
void SetSplit(uint8_t  _band) {
  if (bands[_band].split)
    sendCatRequest(CIV_C_SPLIT_ON_SEND, 0, 0);
  else
    sendCatRequest(CIV_C_SPLIT_OFF_SEND, 0, 0);
}

// set attenuator on radio
void SetAttn(uint8_t  _band) {
  if (bands[_band].atten)
    sendCatRequest(CIV_C_ATTN_ON, 0, 0);
  else
    sendCatRequest(CIV_C_ATTN_OFF, 0, 0);
}

// set preamp on radio
void SetPre(uint8_t  _band) {
  switch (bands[_band].preamp) {
    case 0: sendCatRequest(CIV_C_PREAMP_OFF, 0, 0); break;
    case 1: sendCatRequest(CIV_C_PREAMP_ON, 0, 0); break;
    case 2:
    default: sendCatRequest(CIV_C_PREAMP_ON2, 0, 0);  break;
  };
}

uint8_t getByteResponse(const uint8_t m_Counter, const uint8_t offset, const uint8_t buffer[])
{
  if (m_Counter < offset + 3) {
        return 0;
  }
	uint8_t ret = bcdByte(buffer[offset]) * 100;
  ret += bcdByte(buffer[offset+1]);
  return ret;
}


void setTime(int hr, int min, int sec, int mday, int month, int yr)  // modifed to match arduino TimeLib versionq}
//void setTime(int yr, int month, int mday, int hr, int minute, int sec, int isDst)  // orignal example
{
    setenv("TZ", "UTC", 1);
    tzset();

    tm.tm_year = yr - 1900;   // Set date
    tm.tm_mon = month-1;
    tm.tm_mday = mday;
    tm.tm_hour = hr;      // Set time
    tm.tm_min = min;
    tm.tm_sec = sec;
    //tm.tm_isdst = isDst;  // 1 or 0
    tm.tm_isdst = 0;  // 1 or 0  // setting to 0 for UTC only use
    t = mktime(&tm);
    ESP_LOGI(TAG, "Setting time: %s", asctime(&tm));
    //char *myDate0 = ctime(&t);
    //ESP_LOGI(TAG, "0-myDate: %s", myDate0);
}

//
//  CIV_Action - Takes action on a sucessfully parsed CIV command result from processmessages()
//  Add new CIV commands action here
//  uses the command index from te enum above.  
//
//  To add a new CIV command not in the list already,
//  1. Insert the cmd name (index numbmer) into the enum list any place.
//  2. Clone and edit a new command in the structure, inserted into the same place as the enun list
//  3. add the new switch case statement with the new enum index
//

//#define DBG_CIV1  // command parser entry
#define DBG_CIV2  // just do summary print

void CIV_Action(const uint8_t cmd_num, const uint8_t data_start_idx, const uint8_t data_len, const uint8_t msg_len, const uint8_t *rd_buffer) 
{ 
  #ifdef DBG_CIV1
  ESP_LOGI("CIV_Action", "Entry - cmd = %X  data_start_idx = %d  data_len = %d  rd_buffer:%X %X %X %X %X %X %X %X %x %X %X", cmd_List[cmd_num].cmdData[1], \
             data_start_idx, data_len, rd_buffer[0], rd_buffer[1], rd_buffer[2], rd_buffer[3],rd_buffer[4], rd_buffer[5], rd_buffer[6], \
             rd_buffer[7],rd_buffer[8], rd_buffer[9], rd_buffer[10]);
  #endif
  
  switch (cmd_num) 
  {
    case CIV_C_F_READ:
    case CIV_C_F_SEND:
    case CIV_C_F1_SEND: {
        uint64_t f;
        uint64_t mul;

        if ((data_len == 5 || (radio_address == IC905 && data_len == 6)) && (rd_buffer[4] == 3 || rd_buffer[4] == 0 || rd_buffer[4] == 5)) {

          mul = 1;
          f = 0;
          for (uint8_t i = 5; i < 5 + data_len; i++) {
            if (rd_buffer[i] == 0xFD) continue;  //spike
            f += (rd_buffer[i] & 0x0F) * mul; mul *= 10;  // * decMulti[i * 2 + 1];
            f += (rd_buffer[i] >> 4) * mul; mul *= 10;  //  * decMulti[i * 2];
          }
          //Serial.printf("CIV_Action: Freq %-13llu\n", f);
          read_Frequency(f, data_len);
        }
        break;
    }

    case CIV_C_TX: { // Used to request RX TX status from radio
        if (data_len != 1 || !(rd_buffer[4] == 0x1C && rd_buffer[5] == 0x00))
          break;

        PTT = rd_buffer[6] == 1 ? true : false;
        if (TX_last != PTT)
        {
          if (!use_wired_PTT)        // normally the wired input will pass thru the PTT from radio hardware PTT. 
          //PTT_Output(band, PTT);   //  If that is not available, then use the radio polled TX state .
          //Serial.printf("CIV_Action: TX Status = %d\n", PTT);
          TX_last = PTT;
        }        
        break;                     // Call PTT output here rather than in teh main loop to avoid any loop delay time.
    }

    case CIV_C_MOD_READ:
    case CIV_C_MOD1_SEND:
    case CIV_C_MOD_SEND: {
        ESP_LOGI("CIV_Action MOD_SEND", "Mode Length = %d", data_len);
        // command CIV_C_MODE_READ received
        radio_mode = rd_buffer[data_start_idx];  // CIVresultL.value/100;
        ESP_LOGI("CIV_Action MOD_SEND", "Mode in HEX: %X", radio_mode);
        
        uint8_t i;
        // look up the bcd value in our modelist table to see what radio mode it is 
        for (i = 0; i< MODES_NUM; i++)
        {
          if (modeList[i].mode_num == radio_mode)  // match bcd value to table mode_num value to get out mode index that we store
          {	
            radio_mode = i;  // now know our decimal index
            break;  
          }
        }// radio_mode now converted to a table index
        
        radio_filter = rd_buffer[data_start_idx+1];
        
        //uint8_t i;
        //for (int i = 0; i < MODES_NUM; i++) {
        //  if (modeList[i].mode_num == radio_mode) {   // convert to our list index and also validate number received
        //  }
        //}

        // This command info lacks data mode status so have to use it to trigger extended mode info
        ESP_LOGI("CIV Mode", "***Set flag to get extended mode data after a mode was message received");
        #ifdef GET_EXT_MODE_INFO
          get_ext_mode_flag = true;
        #endif
        
        ESP_LOGI("CIV_Action MOD_SEND", "CIV_Action: CI-V Returned Mode: %s", modeList[i].mode_label);  
        ESP_LOGI("CIV_Action MOD_SEND", "  Mode Index: %d", i); 
        ESP_LOGI("CIV_Action MOD_SEND", "  Filter: %s", FilStr[radio_filter]);    
        break;
    }

    case CIV_C_F26A:
    case CIV_C_F26B:
    case CIV_C_F26: {
		//case CIV_C_F26_SEND:
        if (rd_buffer[data_start_idx] == 0x00) {  // 0 is selected VFO field				 
          // Store the mode, filt an datamode value per band      
          radio_mode = rd_buffer[data_start_idx+1];  // hex value
          radio_data = (rd_buffer[data_start_idx+2] == 1) ? 1 : 0;   // 0-1  limit to 0 or 1 answer
          radio_filter = (rd_buffer[data_start_idx+3] < 4) ? rd_buffer[data_start_idx+3] : 1;  // 1-3, make 1 if a bad value
          
          uint8_t i;
          for (i = 0; i < MODES_NUM; i++) {
            if ((modeList[i].mode_num == radio_mode) && (modeList[i].data == radio_data)) {   // convert to our list index and also validate number received
              bands[band].mode_idx = i;  // mode is in HEX
              break;  // found our match, exit loop
            } 
          }
          
          if (i >= MODES_NUM) {   // if i reaches the top of the list then a match was not found, skip out
            ESP_LOGI("CIV_Action: ERROR: Extended Mode Info", "Invalid Mode: %X", radio_mode);       
            break;
          } else {  // we have a vaid match, finish up.
            bands[band].datamode = radio_data;  // data on/off  0-1
            bands[band].filt = radio_filter;  // filter setting 1-3
            ESP_LOGI("CIV_Action: Extended Mode Info", "ModeNum: %d   Mode: %s   DATA: %s   Filt: %s", bands[band].mode_idx, modeList[bands[band].mode_idx].mode_label, ModeStr[bands[band].datamode], FilStr[bands[band].filt]);
          }
        }  // else the changes are for the unselected VFO				
        break;       
    }

    case CIV_C_UTC_READ_905:
    case CIV_C_UTC_READ_705:  {
        //Serial.printf("processCatMessages: UTC Offset, Len = %d\n", data_len);
        //               pos     sub cmd  len        1    2     3          term
        // FE.FE.E0.AC.  1A.05.  01.81.   3         07.  00.   01.         FD
        // FE.FE.E0.AC.  1A.05.           datalen   hr   min   1=minus 0=+
        //                                     offset time 00 00 to 14 00    Shift dir 00 + and 01 is -
        // when using datafield, add 1 to prog guide index to account for first byte used as length counter - so 3 is 4 here.

        hr_off = bcdByte(rd_buffer[data_start_idx+0]); 
        min_off = bcdByte(rd_buffer[data_start_idx+1]); 
        shift_dir = bcdByte(rd_buffer[data_start_idx+2]);

        if (shift_dir) 
        {
          hr_off = hr_off * -1;  // invert  - used by UTC set function
          min_off = min_off * -1;  // invert  - used by UTC set function
        }
        ESP_LOGI(TAG, "UTC Offset: %d:%d", hr_off, min_off);

        //get current time and correct or set time zone offset
        //setTime(_hr,_min,_sec,_day,_month,_yr);  // apply offset in time set function
        break;
    }  // UTC Offset

    case CIV_C_MY_POSIT_READ: {
        // if (data_len == 23) then altitude is invalid and not sent out, skip
        bool skip_altitude = false;
        if (data_len == 23)
            skip_altitude = true;   // skip 4 bytes when looking for time and date.
        else if (data_len != 27)
          return;
        
        //Serial.printf("processCatMessages: Position, Len = %d\n", data_len);
        //uint8_t RX = CIVresultL.value;

        //ESP_LOGI(TAG, "MY POSITION and TIME: %s", retValStr[CIVresultL.value]);
        //               pos                  1  2  3  4   5     6  7  8  9  10   11       12 13 14 15   16 17   18 19 20   21 22  23  24  25 26 27  term
        // FE.FE.E0.AC.  23.00.  datalen 27  47.46.92.50.  01.   01.22.01.98.70.  00.      00.15.59.00.  01.05.  00.00.07.  20.24. 07. 20. 23.32.45. FD
        //                                   47,46,92,40,  01,   01,22,01,99,60,  00,                    00,58,  00,01,09,  20,24, 08, 28, 11,07,41  FD
        //                                                                                 00,15,64,00,  00,72,  00,00,20
        //                               lat 4746.9250   01(N)     12201.9870    00(-1) long  155.900m alt  105deg   0.7km/h   2024   07  20  23:32:45 UTC
        // when using datafield, add 1 to prog guide index to account for first byte used as length counter - so 27 is 28 here.
        
        // Extract and Process Lat and Long for Maidenhead Grid Square stored in global Grid_Square[]
        //char GPS_Msg[NMEA_MAX_LENGTH] = {}; // {"4746.92382,N,12201.98606,W\0"};
        //uint8_t i = 0;
        uint8_t k = 0;
        float lati_deg = 0.0;
        float longi_deg = 0.0;
        float lati_min = 0.0;
        float longi_min = 0.0;
        float lati = 0.0;
        float longi = 0.0;
        char temp_str[14];

        // reformat latitude
        // 47.46.93.10.  01.  
        lati_deg  = (float) bcdByte(rd_buffer[data_start_idx+k++]);         //  47 deg
        lati_min  = (float) bcdByte(rd_buffer[data_start_idx+k++]);         //  46. min
        lati_min +=  (float) bcdByte(rd_buffer[data_start_idx+k++]) /100;    //    .93
        lati_min += (float) bcdByte(rd_buffer[data_start_idx+k++]) /10000;  //    .0010  last nibble is always 0
        lati_min /= 60;
        lati = lati_deg+lati_min;
        ftoa(lati, temp_str, 6);   // convert to string now, ftoa cannot handle negative numbers
        // if 1 then North, else south will be negative
        if (!bcdByte(rd_buffer[data_start_idx+k++])) /*  use N/S to set neg or pos */              
          sniprintf(Latitude, 11, "%c%s", '-', temp_str);       /*  the ftoa function does not handle neg numbers */
        else
          sniprintf(Latitude, 11, "%c%s", ' ', temp_str);
        
        //Serial.printf("CIV_Action: Latitude Converted to dd mm ss format: Deg=%f  min=%f  lat=%f  string=%s\n",lati_deg, lati_min, lati, Latitude);

        // Longitude  01.22.01.98.70.  00.
        longi_deg  = (float) bcdByte(rd_buffer[data_start_idx+k++])*100;     // 100  first nibble is always 0
        longi_deg += (float) bcdByte(rd_buffer[data_start_idx+k++]);         //  22 deg
        longi_min  = (float) bcdByte(rd_buffer[data_start_idx+k++]);         //  01 min
        longi_min += (float) bcdByte(rd_buffer[data_start_idx+k++]) /100;    //    .98
        longi_min += (float) bcdByte(rd_buffer[data_start_idx+k++]) /10000;  //    .0070
        longi_min /= 60;  // convert minutes to mm.mmm
        longi = longi_deg+longi_min;  // make 1 number
        ftoa(longi, temp_str, 9);   // convert to string now, ftoa cannot handle negative numbers
        
        // if 1 then E, else W will be negative
          if (!bcdByte(rd_buffer[data_start_idx+k++]))
          sniprintf(Longitude, 13, "%c%s", '-', temp_str);        /*  the ftoa function does not handle neg numbers */    
        else
          sniprintf(Longitude, 13, "%c%s", ' ', temp_str); 
        
        //ESP_LOGI(TAG, "CIV_Action: Longitude Converted to dd mm ss format:  deg=%f   min=%f   long=%f  string=%s  %s",longi_deg, longi_min, longi, Latitude, Longitude);

        // if using NMEA string then proved a formnatted string like belw and convert to minutes  
        //strcpy(GPS_Msg, "4746.92382,N,12201.98606,W\0"};   // test string
        //ConvertToMinutes(GPS_Msg);       
        // Here I directly converted to what Convert_to_MH wants
        Convert_to_MH();
        //ESP_LOGI(TAG, "CIV_Action: GPS Converted: Lat = %s  Long = %s  Grid Square is %s", Latitude, Longitude, Grid_Square);
        //ESP_LOGI(TAG, "CIV_Action: ** Time from Radio is: ");

        uint8_t d_index = 0;
        if (skip_altitude) 
            d_index = 4;

        int _hr = bcdByte(rd_buffer[data_start_idx+24-d_index]); //Serial.print(_hr); ESP_LOGI(TAG, ":");
        int _min = bcdByte(rd_buffer[data_start_idx+25-d_index]); //Serial.print(_min);ESP_LOGI(TAG, ":");
        int _sec = bcdByte(rd_buffer[data_start_idx+26-d_index]); //Serial.print(_sec);ESP_LOGI(TAG, " ");
        
        int _month = bcdByte(rd_buffer[data_start_idx+22-d_index]); //Serial.print(_month);ESP_LOGI(TAG, ".");
        int _day = bcdByte(rd_buffer[data_start_idx+23-d_index]); //Serial.print(_day); ESP_LOGI(TAG, ".");
        int _yr = bcdByte(rd_buffer[data_start_idx+21-d_index]); //Serial.print(_yr); // yr can be 4 or 2 digits  2024 or 24                    
        
        if (!UTC) 
        {
          setTime(_hr+hr_off,_min+min_off,_sec,_day,_month,_yr);  // correct to local time                      
          ESP_LOGI(TAG, "Local Time: %d:%d:%d  %d/%d/20%d",_hr+hr_off,_min+min_off,_sec,_month,_day,_yr); 
        }
        else
        {
          setTime(_hr,_min,_sec,_day,_month,_yr);  // display UTC time
          //ESP_LOGI(TAG, "UTC Time: %d:%d:%d  %d/%d/%d",_hr,_min,_sec,_month,_day,_yr); 
        }
        break;
      /*
        %a Abbreviated weekday name 
        %A Full weekday name 
        %b Abbreviated month name 
        %B Full month name 
        %c Date and time representation for your locale 
        %d Day of month as a decimal number (01-31) 
        %H Hour in 24-hour format (00-23) 
        %I Hour in 12-hour format (01-12) 
        %j Day of year as decimal number (001-366) 
        %m Month as decimal number (01-12) 
        %M Minute as decimal number (00-59) 
        %p Current locale’s A.M./P.M. indicator for 12-hour clock 
        %S Second as decimal number (00-59) 
        %U Week of year as decimal number,  Sunday as first day of week (00-51) 
        %w Weekday as decimal number (0-6; Sunday is 0) 
        %W Week of year as decimal number, Monday as first day of week (00-51) 
        %x Date representation for current locale 
        %X Time representation for current locale 
        %y Year without century, as decimal number (00-99) 
        %Y Year with century, as decimal number 
        %z %Z Time-zone name or abbreviation, (no characters if time zone is unknown) 
        %% Percent sign 
        You can include text literals (such as spaces and colons) to make a neater display or for padding between adjoining columns. 
        You can suppress the display of leading zeroes  by using the "#" character  (%#d, %#H, %#I, %#j, %#m, %#M, %#S, %#U, %#w, %#W, %#y, %#Y) 
      */

    }  // MY Position

    case CIV_C_PREAMP_READ: {
        uint8_t _val = rd_buffer[data_start_idx];

        //ESP_LOGI(TAG, "CIV_Action:  CI-V Returned Preamp status: "); ESP_LOGI(TAG, _val);
        if (_val > 0)
        {
            //bands[band].atten = ATTN_OFF;
            bands[band].preamp = _val;              
        }
        if (_val == 0)
        {
            bands[band].preamp = PREAMP_OFF;
        }
        //displayAttn();
        //displayPreamp();
        break;
    }
    
    case CIV_C_SPLIT_READ: {
        uint8_t _val = rd_buffer[data_start_idx];

        //ESP_LOGI(TAG, "CIV_Action:  CI-V Returned Preamp status: "); ESP_LOGI(TAG, _val);
        if (_val > 0)
        {
            //bands[band].atten = ATTN_OFF;
            bands[band].split = 1;              
        }
        if (_val == 0)
        {
            bands[band].split = 0;
        }
        //displayAttn();
        //displayPreamp();
        break;
    }

    case CIV_C_ATTN_READ:	
		case CIV_C_ATTN_ON:
    case CIV_C_ATTN_OFF: {
        uint8_t _val = rd_buffer[data_start_idx];

        //ESP_LOGI(TAG, "CIV_Action:  CI-V Returned Attn status: "); ESP_LOGI(TAG, _val);
        if (_val > 0)
        {
            //bands[band].preamp = PREAMP_OFF;
            bands[band].atten = ATTN_ON;
        }
        if (_val == 0)
        {
            bands[band].atten = ATTN_OFF;
        }
        //displayAttn();
        //displayPreamp();
        break;
    }  // Attn changed

		//case CIV_C_AGC_FAST:
		//case CIV_C_AGC_MID:
		//case CIV_C_AGC_SLOW:
    case CIV_C_AGC_READ: {
        uint8_t _val = rd_buffer[data_start_idx];
        bands[band].agc = _val;
        //displayAgc();
        //ESP_LOGI(TAG, "CIV_Action:  AGC mode: "); ESP_LOGI(TAG, AgcStr[_val]);
        break;
		}  // AGC changed

		case CIV_C_DUPLEX_READ:
		case CIV_C_DUPLEX_SEND: {
        //              Dup            1k/100Hz   100K/10Khz    10M/1MHz    term
        // FE.FE.E0.AC. 0C.               01.        81.          00.        FD
        //					 datalen=3   
        // when using datafield, add 1 to prog guide index to account for first byte used as length counter - so 3 is 4 here.

        //uint8_t DUP_10MHZ 	= bcdByte(CIVresultL.datafield[3] & 0xF0); 
        //uint8_t DUP_1MHZ 	= bcdByte(CIVresultL.datafield[3] & 0x0F); 
        
        //uint8_t DUP_100KHZ 	= bcdByte(CIVresultL.datafield[2] & 0xF0); 
        //uint8_t DUP_10KHZ 	= bcdByte(CIVresultL.datafield[2] & 0x0F); 

        //uint8_t DUP_1KHZ 	= bcdByte(CIVresultL.datafield[1] & 0xF0);
        //uint8_t DUP_100HZ 	= bcdByte(CIVresultL.datafield[1] & 0x0F);
             
        //int8_t DUP_MINUS 	= CIVresultL.datafield[3];    // 00 = plus, 01 - minus
        radio_DUP 	   = bcdByte(rd_buffer[data_start_idx+2])*1000;
        radio_DUP  	  += bcdByte(rd_buffer[data_start_idx+1])*10; 
        radio_DUP  	  += bcdByte(rd_buffer[data_start_idx]); 
        radio_DUP 	  *= 1000;  //convert KHz to Hz
        //radio_DUP = DUP_MINUS ?  radio_DUP*-1: radio_DUP;
        ESP_LOGI(TAG, "CIV_Action:  Radio Returned Duplex Offset: %luHz", radio_DUP);
        break;
		}  // Duplex Offset


		case CIV_C_RIT_XIT: {
        // RIT -1.97KHz
        //              RIT            1k/100Hz   10/1hz   +/1    term
        // FE.FE.E0.AC. 21.00              01.      97.     01.     FD
        //					 datalen[0]=3   
                  //                                     
        // when using datafield, add 1 to prog guide index to account for first byte used as length counter - so 3 is 4 here.

        int8_t RIT_MINUS = rd_buffer[data_start_idx+2];    // 00 = plus, 01 - minus
        radio_RIT 		 = bcdByte(rd_buffer[data_start_idx+1])* 100; // * -RIT_MINUS; 
        radio_RIT  	    += bcdByte(rd_buffer[data_start_idx]); 
        radio_RIT = RIT_MINUS ?  radio_RIT*-1: radio_RIT;
        ESP_LOGI(TAG, "CIV_Action:  RIT/XIT Offset: %d", radio_RIT); ESP_LOGI(TAG, "Hz");
        break;
    }  // RIT/XIT Offset

		case CIV_C_RIT_ON_OFF: {
      	radio_RIT_On_Off  	    = bcdByte(rd_buffer[data_start_idx]); 
				ESP_LOGI(TAG, "CIV_Action:  RIT On/Off: %d", radio_RIT_On_Off);
				break;
		}  // RIT On/Off

		case CIV_C_XIT_ON_OFF: {					
        radio_XIT_On_Off  	    = bcdByte(rd_buffer[data_start_idx]); 
        ESP_LOGI(TAG, "CIV_Action:  XIT On/Off: %d", radio_XIT_On_Off);
        break;
		}  // XIT On/Off

    default: ESP_LOGI(TAG, "CIV_Action: *** default action"); break;
          //knowncommand = false;
  }
  #ifdef DBG_CIV2
    ESP_LOGI("CIV_Action","Radio Status %13llu   band = %s   %7s  %11s  %5s  %6s  PRE:%d  ATT:%d  PTT:%d", \
          frequency, bands[band].band_name, modeList[radio_mode].mode_label, ModeStr[radio_data], FilStr[radio_filter], \
          AgcStr[bands[band].agc], bands[band].preamp, bands[band].atten, PTT);
          
    //      frequency, bands[band].band_name, modeList[bands[band].mode_idx].mode_label, ModeStr[bands[band].datamode], FilStr[bands[band].filt],
    //      AgcStr[bands[band].agc], bands[band].preamp, bands[band].atten, PTT);
    
  #endif
}

