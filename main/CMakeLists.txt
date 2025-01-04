idf_component_register(
     SRCS  "IC905_ESP32-S3_PTT_Breakout.c" "CIV.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb esp_driver_gpio
    )
