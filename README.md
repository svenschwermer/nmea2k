# nmea2k

## Board
- Add additional board manager URL: `https://dl.espressif.com/dl/package_esp32_index.json`
- Add board `esp32` using the board manager
- Select board `ESP32 Dev Module`

## Schematics
- BME280 SCL: GPIO22
- BME280 SDA: GPIO21
- DS1603L RX: UART2 RX (GPIO16)
- CAN TX: GPIO5
- CAN RX: GPIO4
- DS18B20: GPIO15

## Dependencies
- Adafruit BME280 Library (library manager)
- https://github.com/ttlappalainen/NMEA2000/archive/f49a25ae7fd2ab5f5efcf95c78cecf9395c961cb.zip
- https://github.com/ttlappalainen/NMEA2000_esp32/archive/b6ff3b745a9755a36c821eb21a3450b5269be5a8.zip
- OneWire
- DallasTemperature
