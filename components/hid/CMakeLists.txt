idf_component_register(SRCS "hid_host.c"
                       "hid_usage_keyboard.c"
                       "hid_usage_mouse.c"
                       INCLUDE_DIRS "include"
                       REQUIRES usb esp_timer freertos)