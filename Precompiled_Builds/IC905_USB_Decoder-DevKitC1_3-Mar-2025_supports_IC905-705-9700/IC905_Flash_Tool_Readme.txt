IC-905 Band Decoder Flash Download Tool image files
(Works with IC-705, IC-905, and IC-9700)

This should also work with any Icom USB VHF+ radio, particularly the IC-9700.

It will auto-detect the radio CI-V address and accept any USB VID/PID.

IC-705 should work as long as you stay on 144 and 430 bands.  There is no support for the HF/6M bands yet and weird results may happen.

This file set is tested on the IC-905 and a ESP32-S3-DevKitC-1 N16R8 clone board.  

To flash firmware to your ESP32-S3 board, follow the instructions located on the project GitHub site Wiki pages.

https://github.com/K7MDL2/IC-905_Band_Decoder_on_ESP32-S3/wiki/Using-Flash-Download-Tool


There are 3 files plus this readme file.  The filenames are listed below along with the offset number that you will enter with it in the Flash Download Tool UI.

1. Bootloader.bin   0x0
2. IC905_ESP32-S3_PTT_Breakout.bin  0x10000  (1 plus 4 zeros)
3. partition-table.bin 0x8000 (8plus 3 zeros)
