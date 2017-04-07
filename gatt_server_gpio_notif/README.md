# gatt_server_gpio_notif
A sample program for ESP32 on esp-idf. This program do the following behaviors.
- Send notification by push or release GPIO0 switch
- Turn on or off LED that connected to GPIO5 by writing 0 or 1 over GATT.

# Partner program
The following nodejs program can listen notice from and write data for this project.

[esp32-nodejs-samples/wirite-and-listen.js](https://github.com/asukiaaa/esp32-nodejs-samples/blob/master/wirite-and-listen.js)

# License
Appache v2

# References
- [esp-idf/examples/bluetooth/gatt_server](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/gatt_server)
- [esp-idf/examples/peripherals/gpio/](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/gpio)
- [GATT SERVER API: esp_ble_gatts_send_indicate](http://esp-idf.readthedocs.io/en/latest/api/bluetooth/esp_gatts.html#_CPPv227esp_ble_gatts_send_indicate13esp_gatt_if_t8uint16_t8uint16_t8uint16_tP7uint8_tb)
