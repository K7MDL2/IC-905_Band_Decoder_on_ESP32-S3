| Support Targets | ![alt text][esp32s3] |
| --- | --- |

| Boards Used | ![alt text][esp32s3DevKitC1] | ![alt text][M5AtomS3] |
| --- | --- | --- |

[esp32s3]: https://img.shields.io/badge/-ESP32_S3-green "ESP32-S3"
[esp32s3DevKitC1]: https://img.shields.io/badge/-ESP32_S3_DevKitC1-blue "ESP32s3DevKitC-1"
[M5AtomS3]: https://img.shields.io/badge/-M5AtomS3-orange "M5AtomS3"

25 Jan 2025 -  Received remaining needed parts except for a pair of 3-to-8 line decoder chips on the Remote board I forgot to order.  The main concern was the pluggable terminal blocks, which have captivation screws, woudl properly fit 

![20250123_123705](https://github.com/user-attachments/assets/439ef68b-d986-4bab-92c9-cd592de17d4c)

![20250125_160846](https://github.com/user-attachments/assets/0a110110-9889-4ad0-b90d-9c34388d8dfe)


24 Jan 2024 - Uploaded full BOMs and PCB gerber files and schematic PDFs.

23 Jan 2025 -  20 sets of PCBs arrived today early.   Assembled and tested the main board.  Only missing 1 4-pin connector and resettable fuse for 24V option, and 1 resistor network for the LEDs.  Used an axial resistor poked into the holes to test the LEDs.  Everything is working as designed.  The missing parts should arrive Jan 25th.  Assembled the Remote Decoder board minus the chips and edge connectors which are arriving Jan 25th.  12V and 5V power and LED working.

Updated the firmare for the new GPIO pin assignments.  The Hammond enclosure and the other parts should be here in 2 days  I have some 1/2" long LED light pipes coming to try out so I mounted the LEDs onto the PCB.

https://github.com/K7MDL2/IC-905_Band_Decoder_on_ESP32-S3/wiki/PCB-Version


Below are recent Dev and prptotype Wiki page links:

https://github.com/K7MDL2/IC-905_Band_Decoder_on_ESP32-S3/wiki/Development-Notes

The prototype construction is documented on this page: https://github.com/K7MDL2/IC-905_Band_Decoder_on_ESP32-S3/wiki/Prototype-Construction

There are now precompiled builds available for the DevKitC and M5AtomS3.  No dev tools or programming knowledge required.  See the Wiki page about how to flash firmware to your ESP32 CPU module with the Windows-based Flash Download Tool: https://github.com/K7MDL2/IC-905_Band_Decoder_on_ESP32-S3/wiki/Using-Flash-Download-Tool

Other pages are or will be created for build, design, and operate. The Wiki Home page is at https://github.com/K7MDL2/IC-905_Band_Decoder_on_ESP32-S3/wiki for more detail.

# USB Band Decoder for the IC-905

This is a Band Decoder and PTT Breakout for the IC-905 radio.  It plugs into the IC-905 USB port and communicates with the radio using CI-V serial protocol to extract frequency (for band), time, date, location (for grid square calculation), and extended mode info (data and voice mode).  It now auto-detects the radio's CI-V address at startup and works with any valid USB VID/PID.  So you can plug it into another VHF+ radio model like the IC-9700 and it should work.  Other VHF capable USB equipped models may sort of work as long as you stay on the VHF+ bands.  I expect to add support for HF/6M bands later.  

It provides:
* 6 band outputs for switching antenna relays, enabling RF amps, etc.
* 6 PTT outputs for keying amplifiers
* 1 PTT input (wired to the radio SEND jack)
* 8 dimmable and assignable status LEDs for Power, 6 bands, and PTT Input

![20250110_130504](https://github.com/user-attachments/assets/d7b7fba9-d3f7-45c5-aa3e-9a9ae8cdf506)

![20250114_150523](https://github.com/user-attachments/assets/008623c9-15a1-4478-90a3-d50344b063b8)

By default each band (there are 6) is configured to operate one Band output pin and 1 PTT output pin in TX.  The IC-905 SEND jack pulls the line to +5V in RX, GND in TX. Buffers are used to protect the CPU IO pins against voltages > 3.3V. 

The PTT and Band output pin(s) patterns are configurable to output any pattern within their function groups (band or PTT).  By default a single output is activated per band, same for the PTT output.  A logic 1 at the GPIO gets inverted at the buffer thus closing the buffer output to GND, the most common need for external amplifiers.  Again, this can be easily configured differently in the code.  PTT polarity is also easily changed in the code if needed. Debouncing the PTT Input is not required since the radio has a transistor SEND output switch so it is clean.  The flex_glitch filter does not apply to the S3 but I did apply the fixed glitch filter (2 clocks) in the PTT input.

There are 8 independent LEDs assigned by default to indicate the 6 bands (blue), PTT input (red), and Power ON (green).  I used the S3's 8 LED PWM drivers to make them dimmable and reduce overall power consumption.  A potentiometer on the back panel adjusts the brightness to adapt to bright daylight, darker shack, or in-vehicle conditions.

In a more distant future I may make this work over USB and BLE (Bluetooth Low Energy) for an IC-705 with transverter support.  This already exists in my other projects but this project will have a PCB designed for it, making it a bit more convenient for some.

This code started out with the esp-idf peripherals example for cdc-acm usb host lib.  I added in bits for GPIO and GPIO input with interrupt from other samples and chunks of my other IC-705/IC-905 band decoder projects.  This project differs from my others in that is it intended to be a small box with PCB and narrowly focused on being a basic 6 band decoder.  The others go further with graphics screen, transverter support and flexible IO choices.

Here I have code running on a M5AtomsS3 connected to my IC-905.  It has 1 USB-OTG port and IO can be exended with i2c connected modules.  It is easier to develop on a board with 2 USB ports.  The M5StampS3 has 1 USB C jack but also has the UART bridge port as pins (D+, D-) on the PCB you can wire to a USB connector and have 2 ports.  This can be an option as well.  I tried them all but in the end I chose the larger ESP32-S3-DevKitC-1 dual port module using discrete LEDs for simpler PCB build and more IO pins that were needed.

![20250105_182044](https://github.com/user-attachments/assets/f9c8ed31-90ea-421f-981f-10adf36ac2ac)



## How to use

Connect the USB-UART (labeled COM port on some boards) to your PC.  Connect the USB-OTG port to the IC-905.  Upload precompiled firmware per instructions on the Wiki page (under  construction).  If you can successfully set up the Expressif ESP-IDF extension in Visual Studio Code then you can build this repository locally and upload.  You can use any serial monitor (putty, Arduino, esp-idf) to monitor the debug info on the com port.  The USB host port requires 5V for the radio USB port to see and activate.  More details on that below and in the Wiki pages.

If no radio is available and you just want to check things out (within limits), you can take 2 boards and connect their USB-OTG ports together using a Type C to Type C USB cable.  On one, designated the 'Device", run the tusb-serial-device example code.  On the other run this code.  This code is configured to look for 2 possible USB VID and PID (vendor and Product IDs) so will connect with either a radio or the ESP32-S3 DevKitC-1 device.   Connect the COM port of each board to your PC and open a terminal on each to see them talk.  

Connected to a radio, when things are working you will see the debug on the COM port with the correct frequency info from the radio and you can put a voltmeter on the board IO pins and see 0V and 3.3V per the pin assignments.


### Hardware Required

You will need one ESP32-S3 board which is capable of USB-OTG support.  Some other boards support USB-OTG as well. The easiest way to develop and test is to use a model with 2 USB ports, one port for OTG (operating in USB Host mode) and the other port (USB-UART) to program and debug the board.

For dev I used 2 board models, one with Host 5V and a 4.3" LCD screen, the other a small module with no screen and you need to have a Y cable to supply +5VDC on the USB host bus port. You can modify the USB host cable or jumper 5V on the CPU module PCB.

What I used:

ESP32-S3-DevKit N16R8 Development Board - small module with 2 ports. About $7 each.  The PSRAM is not used so other models with smaller or no PSRAM will work.  You need to add 5V with a Y cable or jumper 5V to the USB host port connector to power the client device (radio or another test board).  There are many CPU variations that will work, most important feature is the ESP32-S3 and 2 USB ports, one OTG.

https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitm-1/user_guide.html#getting-started

https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide.html

These have no screen and no Type A host port, but still have 2 USB ports, one is OTG.  Has WiFi and Bluetooth Low Energy (BLE).

I have this one on order and plan to test with.  ESP32-S3-USB-OTG Development Board.  About $35.

https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-usb-otg/user_guide.html

This uses a ESP32-S3-Mini module stacked on a like-sized motherboard with a 1.3” LCD and 2 USB connectors. One is a USB Type A female host that provides 5V@500ma and the other a USB Type A male to connect to a PC.

I have M5Stack M5StampS3 and M5StackC3U CPU modules, about $7 each, they support USB OTG but ony come with 1 USB port connector, a 2nd port can be wired to your own connector. USing only 1 USB port, once the code is uploaded over the single OTG port in boot mode, it will run the new firmware and go into OTG host mode to talk to the radio.

I have an 800x480 LCD ESP32-S3 OTG dev kit version from Waveshare with 2 USB ports, one 5V powered OTG, that works well also since it provides the needed 5V.  About $35.

I have a configuration option to run on the M5stack M5AtomS3 leveraging its color LCD screen.  It has a 0.95" color LCD, one OTG port. It looks pretty good and is very small yet readable, I use it on my 705 decoder. Adding i2c connector IO expansion is easy, just a 4-wire cable that plugs in.  The hard part is how to mount it in a box.  It works but is messy to add all the IO we need.

The advantage of the boards that provide built-in 5V is they are, or should be, done properly with a fuse and/or current limiting device so you do not cook the client device in a mishap.  The biggest feature is no Y cable!  I used a 1K resistor to jumper the DevKitC-1 module +5 to the host USB.  See the Wiki pages on how I did this.

Speaking of cables, do yourself a favor and inventory your Type C cables.  C to C and A to C.  Test them to be sure they are not charging-only cables!  Mark them if they charging only, it will save you grief!   I found I had 2 of these I was using and spent hours fixing unbroken or partially broken software. Easily missed if you are switching cables a lot while testing and do not have working software to star with and thus know what to blame.  I also lacked 18" cables.  They are all 6" or 6ft.  Creates a cable mess. I ordered 3-packs of 18" cables, much better on my desktop now.  


#### Pin Assignment

The gpio pins are defined in the decoder.h and LED_Control.h files. There are 22 IO pins used.  1 for PTT input, 6 Band decode outputs, 6 PTT outputs, 8 LEDs, and 1 ADC reading a pot position.  You can change the pin numbers no problem as long as they are not used for something else. There is no need to change any other code, the pin assignments are abstracted.  This code has default pin assignments to match a PCB pinout.


### Build and Flash

Build this project using the esp-idf extension in VS Code and flash it to the USB OTG host board, then run monitor tool to view serial output. There are some settings that have to be made located in the sdkconfig file for hub support and packet size.  You can edit it or use menuconfig tool.  I have esp-idf setup info in a Wiki page.  There is a specific config order recommended to avoid errors.

Below are OTG related sdkconfig settings. I modify or enable these to support USB hub and the larger than default descriptor sizes.  They are sometimes lost if you start the Setup Configure Extension wizard with setting the ESP32-S3 first.  I included in the repository here 'sdkconfig.defaults' file which has these.  In theory it will help ensure you do not lose these values.

      CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE = 2048   --> if you see errors this is usually the culprit, default of 256 is too small for hubs.
      CONFIG_USB_HOST_HUBS_SUPPORTED=y     --> If you see you cannot open the radio and it is connected, this is likely not set
      CONFIG_USB_OTG_SUPPORTED=y   --> usually already set.

      #
      # USB-OTG
      #
      CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=2048
      CONFIG_USB_HOST_HW_BUFFER_BIAS_BALANCED=y
      # CONFIG_USB_HOST_HW_BUFFER_BIAS_IN is not set
      # CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT is not set
      
      #
      # Hub Driver Configuration
      #
      
      #
      # Root Port configuration
      #
      CONFIG_USB_HOST_DEBOUNCE_DELAY_MS=250
      CONFIG_USB_HOST_RESET_HOLD_MS=30
      CONFIG_USB_HOST_RESET_RECOVERY_MS=30
      CONFIG_USB_HOST_SET_ADDR_RECOVERY_MS=10
      # end of Root Port configuration
      
      CONFIG_USB_HOST_HUBS_SUPPORTED=y
      CONFIG_USB_HOST_HUB_MULTI_LEVEL=y
      
      #
      # Downstream Port configuration
      #
      CONFIG_USB_HOST_EXT_PORT_RESET_RECOVERY_DELAY_MS=30
      # CONFIG_USB_HOST_EXT_PORT_CUSTOM_POWER_ON_DELAY_ENABLE is not set
      # end of Downstream Port configuration
      # end of Hub Driver Configuration
      
      # CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK is not set
      CONFIG_USB_OTG_SUPPORTED=y
      # end of USB-OTG


If using one board for a device rather than the radio, build and flash [tusb_serial_device example](../../../device/tusb_serial_device) to USB device board.  It does not have to be a OTG capable board, just a ESP32 with 1 free USB port.

See the ESP-IDF Getting Started Guide for full steps to configure and use ESP-IDF to build projects.


## Example Output

After the flashing you should see the output at idf monitor:

```
...
I (256) USB-CDC: USB Host installed
I (256) USB-CDC: Opening CDC ACM device 0x303A:0x4001
...
Device descriptor is printed here
...
I (1666) USB-CDC: Data received
I (1666) USB-CDC: 0x3ffc4c20   41 54 0d                                          |AT.|
I (2666) USB-CDC: Data received
I (2666) USB-CDC: 0x3ffc4c20   41 54 2b 47 53 4e 0d                              |AT+GSN.|
I (3666) USB-CDC: Setting up line coding
I (3666) USB-CDC: Line Get: Rate: 115200, Stop bits: 0, Parity: 0, Databits: 8
I (3666) USB-CDC: Line Set: Rate: 9600, Stop bits: 1, Parity: 1, Databits: 7
I (3666) USB-CDC: Line Get: Rate: 9600, Stop bits: 1, Parity: 1, Databits: 7
I (3676) Example finished successfully!
...
Various Hex data and debug messages follow.  

The firmware will poll the radio on startup for time, data, location, time offset, frequency, and mode.
It will also poll the radio for extended mode information when you change bands or modes since the standard mode message omits the Datamode status.

Wiki Pages will contain more detail.

I will be designing a PCB to mount the ESP32-S2-DevKitC-1, 2 ULN2803A buffers, PTT input jack, brightness pot, and 8 LEDs.  It has a 12V to 5V regulator and two 12V 2.1mm x 5.5mm standard coaxial DC power jacks.  A 15 pin connector for the outputs, 12V and 5V.  On the cable side I often use a "breakout" HD15 which has screw terminals on a small PCB.  Easier than soldering the HD15 pins.
