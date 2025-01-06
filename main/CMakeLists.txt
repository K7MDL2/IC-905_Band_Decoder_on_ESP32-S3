idf_component_register(
     SRCS  "IC905_ESP32-S3_PTT_Breakout.cpp" "CIV.cpp"
    INCLUDE_DIRS "." "../managed_components/espressif__usb_host_cdc_acm/include/usb"
    PRIV_REQUIRES usb esp_driver_gpio esp_timer
    )
