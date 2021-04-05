// SPDX-License-Identifier: GPL-3.0-or-later
// Based on the work by Andreas Koritnik (AK-Homberger)
// https://github.com/AK-Homberger/NMEA2000-Workshop

#include <Arduino.h>
#include <Preferences.h> // From esp32 board

// NMEA2000 stuff, see https://github.com/ttlappalainen/NMEA2000
#define ESP32_CAN_TX_PIN GPIO_NUM_5
#define ESP32_CAN_RX_PIN GPIO_NUM_4
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>

// BME280
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// DS1603L
#include "ds1603l.h"

// DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

//*****************************************************************************

Adafruit_BME280 bme; // I2C: SCL=GPIO22 SDA=GPIO21

DS1603L ultrasonic(Serial2);
static const uint16_t tank_min = 100, tank_max = 280; // in millimeters
static const double tank_capacity = 200.0; // in liters

OneWire one_wire(15); // GPIO15
DallasTemperature ds18b20(&one_wire);
uint8_t ds18b20_count = 0;
tN2kTempSource ds18b20_src[] = {
  N2kts_ExhaustGasTemperature,
  N2kts_HeatingSystemTemperature,
};
uint8_t ds18b20_addr[ARRAY_SIZE(ds18b20_src)][8];

int NodeAddress;         // To store last Node Address
Preferences preferences; // Nonvolatile storage on ESP32 - To store LastDeviceAddress

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {
  130312UL, // Temperature
  130313UL, // Humidity
  130314UL, // Pressure
  127505UL, // Fluid level
  0};
static const uint8_t sid_no_position_fix = 0xff;

//*****************************************************************************

void setup()
{
  // Init USB serial port
  Serial.begin(115200);
  delay(10);

  // Init BME280 I2C address depends on sensor 0x76 or 0x77.
  if (!bme.begin(0x76))
    Serial.println("Could not find a valid BME280 sensor, check wiring!");

  Serial2.begin(9600);

  ds18b20.setWaitForConversion(false);
  ds18b20.begin();
  for (bool ok=true; ok && ds18b20_count < ARRAY_SIZE(ds18b20_addr); )
  {
    auto addr = ds18b20_addr[ds18b20_count];
    ok = ds18b20.getAddress(addr, ds18b20_count);
    if (ok)
    {
      Serial.printf("DS18B20 index=%d addr=%02x%02x%02x%02x%02x%02x%02x%02x\n",
        ds18b20_count, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]);
      ++ds18b20_count;
    }
  }
  Serial.printf("DS18B20 count: %d\n", ds18b20_count);
  ds18b20.requestTemperatures();

  // Reserve enough buffer for sending all messages.
  NMEA2000.SetN2kCANMsgBufSize(16);
  NMEA2000.SetN2kCANReceiveFrameBufSize(128);
  NMEA2000.SetN2kCANSendFrameBufSize(1024);

  // Generate unique number from MAC address
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  uint32_t id = mac[3] << 24 | mac[2] << 16 | mac[1] << 8 | mac[0];

  // Set product information
  NMEA2000.SetProductInformation("1",                     // Manufacturer's Model serial code
                                 100,                     // Manufacturer's product code
                                 "My Sensor Module",      // Manufacturer's Model ID
                                 "1.0.2.25 (2019-07-07)", // Manufacturer's Software version code
                                 "1.0.2.0 (2019-07-07)"   // Manufacturer's Model version
  );
  // Set device information
  NMEA2000.SetDeviceInformation(id,  // Unique number. Use e.g. Serial number.
                                132, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25,  // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  preferences.begin("nvs", false);                         // Open nonvolatile storage (nvs)
  NodeAddress = preferences.getInt("LastNodeAddress", 34); // Read stored last NodeAddress, default 34
  preferences.end();
  Serial.printf("NodeAddress=%d\n", NodeAddress);

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, NodeAddress);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);

  NMEA2000.Open();

  delay(200);
}

//*****************************************************************************

bool bme280_present(void)
{
  return bme.sensorID() == 0x60;
}

void SendN2kTemperature(void)
{
  if (!bme280_present())
    return;

  double temperature = bme.readTemperature();
  Serial.printf("Temperature: %3.1f°C\n", temperature);

  tN2kMsg N2kMsg;
  SetN2kTemperature(N2kMsg, sid_no_position_fix, 0, N2kts_InsideTemperature, CToKelvin(temperature));
  NMEA2000.SendMsg(N2kMsg);
}

void SendN2kHumidity(void)
{
  if (!bme280_present())
    return;

  double humidity = bme.readHumidity();
  Serial.printf("Humidity: %3.1f%%\n", humidity);

  tN2kMsg N2kMsg;
  SetN2kHumidity(N2kMsg, sid_no_position_fix, 0, N2khs_InsideHumidity, humidity, N2kDoubleNA);
  NMEA2000.SendMsg(N2kMsg);
}

void SendN2kPressure(void)
{
  if (!bme280_present())
    return;

  double pressure = bme.readPressure() / 100; // Read and convert to mBar
  Serial.printf("Pressure: %3.1f mBar\n", pressure);

  tN2kMsg N2kMsg;
  SetN2kPressure(N2kMsg, sid_no_position_fix, 0, N2kps_Atmospheric, mBarToPascal(pressure));
  NMEA2000.SendMsg(N2kMsg);
}

void HandleDS18B20(void)
{
  if (ds18b20.isConversionComplete())
  {
    ds18b20.requestTemperatures();

    for (uint8_t i=0; i<ds18b20_count; ++i)
    {
      float tempC = ds18b20.getTempC(ds18b20_addr[i]);
      Serial.printf("Temperature (DS18B20 #%d): %3.1f°C\n", i, tempC);

      tN2kMsg N2kMsg;
      SetN2kTemperature(N2kMsg, sid_no_position_fix, 1+i, ds18b20_src[i], CToKelvin(tempC));
      NMEA2000.SendMsg(N2kMsg);
    }
  }
}

void CheckSourceAddressChange()
{
  int SourceAddress = NMEA2000.GetN2kSource();

  if (SourceAddress != NodeAddress)
  {                              // Save potentially changed Source Address to NVS memory
    NodeAddress = SourceAddress; // Set new Node Address (to save only once)
    preferences.begin("nvs", false);
    preferences.putInt("LastNodeAddress", SourceAddress);
    preferences.end();
    Serial.printf("Address Change: New Address=%d\n", SourceAddress);
  }
}

void HandleUltrasonicSensor()
{
  uint16_t reading;
  double percentage;
  tN2kMsg N2kMsg;
  switch (ultrasonic.read(reading)) {
  case DS1603L::state::new_data:
    reading = std::max(std::min(reading, tank_max), tank_min) - tank_min;
    percentage = 100 * static_cast<double>(reading) / static_cast<double>(tank_max - tank_min);
    Serial.printf("New ultrasonic sensor reading: %dmm (%f%%)\n", reading, percentage);
    SetN2kFluidLevel(N2kMsg, 0, N2kft_Fuel, percentage, tank_capacity);
    NMEA2000.SendMsg(N2kMsg);
    break;
  case DS1603L::state::checksum_fail:
    Serial.println("Error: Ultrasonic sensor checksum error");
    break;
  case DS1603L::state::no_input:
    Serial.println("Error: No ultrasonic sensor data");
    break;
  }
}

void loop()
{
  static enum {
    ultrasonic,
    temperature,
    humidity,
    pressure,
    ds18b20,
  } state = ultrasonic;
  static uint32_t last_measurement = millis();

  if (millis() - last_measurement > 250)
  {
    switch (state)
    {
    case ultrasonic:
      HandleUltrasonicSensor();
      state = temperature;
      break;
    case temperature:
      SendN2kTemperature();
      state = humidity;
      break;
    case humidity:
      SendN2kHumidity();
      state = pressure;
      break;
    case pressure:
      SendN2kPressure();
      state = ds18b20;
      break;
    case ds18b20:
      HandleDS18B20();
      state = ultrasonic;
      break;
    }
    last_measurement = millis();
  }

  NMEA2000.ParseMessages();
  CheckSourceAddressChange();

  // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  if (Serial.available())
  {
    Serial.read();
  }
}
