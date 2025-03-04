idf_component_register(
    SRCS "IC905_ESP32-S3_PTT_Breakout.cpp" "CIV.cpp" "LED_Control.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb esp_driver_gpio esp_timer driver esp_driver_ledc
    )
