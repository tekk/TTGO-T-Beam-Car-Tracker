# Car / bike GPS tracker with TTGO-T-Beam & A6 GPRS module
### Using ESP32 + LoRa + A6 GPRS module

## Libraries needed

- [ESP32 Core for Arduino](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md) (installation with Boards Manager)
- [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus)
- [CayenneLPP](https://github.com/sabas1080/CayenneLPP)

## Instructions

You need to connect the [T-Beam](https://github.com/LilyGO/TTGO-T-Beam) DIO1 pin marked *Lora1* to the *pin 33* - So that the ESP32 can read that output from the Lora module.
Optionally you can also connect the *Lora2* output to *GPIO 32*, but this is not needed here.

You can program the T-Beam using the [Arduino ESP32](https://github.com/espressif/arduino-esp32) board 'Heltec_WIFI_LoRa_32'.

### Update `config.h`

Update with your own [TTN keys](https://www.thethingsnetwork.org/docs/devices/registration.html).

## Reference

- **TTGO-T-Beam**

![TTGO-T-Beam](images/ttgo-t-beam.jpg)

- **A6 GSM/GPRS module**

![A6 GSM/GPRS](images/a6-gsm-gprs-dev-board.jpg)

## Further improvement

You are welcome to contribute to this project in any way. Suggestions/feedback is much appreciated.

## Credits

- Thanks to DeuxVis for his [Lora-TTNMapper-T-Beam](https://github.com/DeuxVis/Lora-TTNMapper-T-Beam)