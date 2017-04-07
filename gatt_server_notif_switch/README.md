# gatt_server_notif_switch
A sample program for ESP32 on esp-idf. This program send notification by push or release GPIO0 switch.

# Partner program
The following nodejs program can listen notice from this project.

[esp32-nodejs-samples/listen-notification.js](https://github.com/asukiaaa/esp32-nodejs-samples/blob/master/listen-notification.js)

# License
Appache v2

# References
- [esp-idf/examples/bluetooth/gatt_server](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/gatt_server)
- [esp-idf/examples/peripherals/gpio/](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/gpio)
- [GATT SERVER API: esp_ble_gatts_send_indicate](http://esp-idf.readthedocs.io/en/latest/api/bluetooth/esp_gatts.html#_CPPv227esp_ble_gatts_send_indicate13esp_gatt_if_t8uint16_t8uint16_t8uint16_tP7uint8_tb)
- [ESP32からBLE GATTのnotifを発信し、nodejs(noble)で受信する方法](http://asukiaaa.blogspot.com/2017/04/esp32ble-gattnodejsnoble.html)