#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <PZEM004Tv30.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <BH1750.h>

constexpr int PZEM_RX_PIN = 16;
constexpr int PZEM_TX_PIN = 17;
constexpr int PZEM2_tx = 27;
constexpr int PZEM2_rx = 26;

constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;
constexpr uint8_t TCA9548A_MIN_ADDRESS = 0x70;
constexpr uint8_t TCA9548A_MAX_ADDRESS = 0x77;
constexpr uint8_t MLX90614_ADDRESS = 0x5A;
constexpr uint8_t BH1750_ADDRESS = 0x23;
constexpr uint8_t MLX90614_A_CHANNEL = 6;
constexpr uint8_t MLX90614_B_CHANNEL = 4;
constexpr uint8_t BH1750_A_CHANNEL = 5;
constexpr uint8_t BH1750_B_CHANNEL = 3;

constexpr uint8_t RECEIVER_CHANNEL = 1;

uint8_t ESP32S3_MACADDRESS[] = {0x90, 0xE5, 0xB1, 0xD1, 0x62, 0x58};

PZEM004Tv30 pzem(Serial1, PZEM_RX_PIN, PZEM_TX_PIN);
PZEM004Tv30 pzem2(Serial2, PZEM2_rx, PZEM2_tx);
Adafruit_MLX90614 mlxA;
Adafruit_MLX90614 mlxB;
BH1750 bh1750A(BH1750_ADDRESS);
BH1750 bh1750B(BH1750_ADDRESS);
uint8_t tca9548aAddress = 0;
bool tca9548aDetected = false;
bool mlxAReady = false;
bool mlxBReady = false;
bool bhAReady = false;
bool bhBReady = false;

typedef struct struct_msg {
  float voltageA;
  float currentA;
  float powerA;
  float energyA;
  float frequencyA;
  float powerFactorA;
  float voltageB;
  float currentB;
  float powerB;
  float energyB;
  float frequencyB;
  float powerFactorB;
  float ambientTempA;
  float objectTempA;
  float ambientTempB;
  float objectTempB;
  float lightLuxA;
  float lightLuxB;
} struct_msg;

struct_msg myData;

esp_now_peer_info_t recvInfo;

bool detectTca9548a()
{
  for (uint8_t address = TCA9548A_MIN_ADDRESS;
       address <= TCA9548A_MAX_ADDRESS;
       ++address)
  {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0)
    {
      tca9548aAddress = address;
      tca9548aDetected = true;
      Serial.printf("TCA9548A detected at I2C address 0x%02X\n", address);
      return true;
    }
  }

  tca9548aAddress = 0;
  tca9548aDetected = false;
  Serial.println("TCA9548A NOT detected (checked addresses 0x70-0x77)");
  return false;
}

bool tcaSelectChannel(uint8_t channel)
{
  if (!tca9548aDetected || channel > 7)
  {
    return false;
  }

  Wire.beginTransmission(tca9548aAddress);
  Wire.write(static_cast<uint8_t>(1 << channel));
  return Wire.endTransmission() == 0;
}

void tcaDisableChannels()
{
  if (!tca9548aDetected)
  {
    return;
  }

  Wire.beginTransmission(tca9548aAddress);
  Wire.write(0);
  Wire.endTransmission();
}

float readAmbientTemperature(Adafruit_MLX90614 &sensor)
{
  float value = sensor.readAmbientTempC();
  return isnan(value) ? NAN : value;
}

float readObjectTemperature(Adafruit_MLX90614 &sensor)
{
  float value = sensor.readObjectTempC();
  return isnan(value) ? NAN : value;
}

bool initializeMuxSensors()
{
  if (!detectTca9548a())
  {
    Serial.println("I2C sensors disabled; PZEM and ESP-NOW will continue");
    return false;
  }

  if (tcaSelectChannel(MLX90614_A_CHANNEL))
  {
    mlxAReady = mlxA.begin(MLX90614_ADDRESS, &Wire);
    if (!mlxAReady)
    {
      Serial.printf("MLX-A not detected on TCA channel %u at 0x5A\n",
                    static_cast<unsigned>(MLX90614_A_CHANNEL));
    }
  }
  else
  {
    Serial.printf("TCA9548A channel %u select failed; check mux address/wiring\n",
                  static_cast<unsigned>(MLX90614_A_CHANNEL));
  }

  if (tcaSelectChannel(MLX90614_B_CHANNEL))
  {
    mlxBReady = mlxB.begin(MLX90614_ADDRESS, &Wire);
    if (!mlxBReady)
    {
      Serial.printf("MLX-B not detected on TCA channel %u at 0x5A\n",
                    static_cast<unsigned>(MLX90614_B_CHANNEL));
    }
  }
  else
  {
    Serial.printf("TCA9548A channel %u select failed; check mux address/wiring\n",
                  static_cast<unsigned>(MLX90614_B_CHANNEL));
  }

  if (tcaSelectChannel(BH1750_A_CHANNEL))
  {
    bhAReady = bh1750A.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_ADDRESS, &Wire);
    if (!bhAReady)
    {
      Serial.printf("BH1750-A not detected on TCA channel %u at 0x23\n",
                    static_cast<unsigned>(BH1750_A_CHANNEL));
    }
  }
  else
  {
    Serial.printf("TCA9548A channel %u select failed; check mux address/wiring\n",
                  static_cast<unsigned>(BH1750_A_CHANNEL));
  }

  if (tcaSelectChannel(BH1750_B_CHANNEL))
  {
    bhBReady = bh1750B.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_ADDRESS, &Wire);
    if (!bhBReady)
    {
      Serial.printf("BH1750-B not detected on TCA channel %u at 0x23\n",
                    static_cast<unsigned>(BH1750_B_CHANNEL));
    }
  }
  else
  {
    Serial.printf("TCA9548A channel %u select failed; check mux address/wiring\n",
                  static_cast<unsigned>(BH1750_B_CHANNEL));
  }

  tcaDisableChannels();

  Serial.printf("I2C sensors: MLX-A=%s MLX-B=%s BH1750-A=%s BH1750-B=%s\n",
                mlxAReady ? "OK" : "FAIL",
                mlxBReady ? "OK" : "FAIL",
                bhAReady ? "OK" : "FAIL",
                bhBReady ? "OK" : "FAIL");

  return mlxAReady || mlxBReady || bhAReady || bhBReady;
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  initializeMuxSensors();

  // PZEM-004T modules need a short startup period after ESP32 power-on.
  Serial1.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
  Serial2.begin(9600, SERIAL_8N1, PZEM2_rx, PZEM2_tx);
  delay(2000);

  Serial1.flush();
  Serial2.flush();

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW initialization failed");
    return;
  }

  esp_now_register_send_cb(esp_now_send_cb_t(onDataSent));

  memcpy(recvInfo.peer_addr, ESP32S3_MACADDRESS, 6);
  recvInfo.channel = RECEIVER_CHANNEL;
  recvInfo.encrypt = false;

  if (esp_now_add_peer(&recvInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  myData.voltageA = pzem.voltage();
  myData.currentA = pzem.current();
  myData.powerA = pzem.power();
  myData.energyA = pzem.energy();
  myData.frequencyA = pzem.frequency();
  myData.powerFactorA = pzem.pf();

  myData.voltageB = pzem2.voltage();
  myData.currentB = pzem2.current();
  myData.powerB = pzem2.power();
  myData.energyB = pzem2.energy();
  myData.frequencyB = pzem2.frequency();
  myData.powerFactorB = pzem2.pf();

  myData.ambientTempA = NAN;
  myData.objectTempA = NAN;
  myData.ambientTempB = NAN;
  myData.objectTempB = NAN;
  myData.lightLuxA = NAN;
  myData.lightLuxB = NAN;

  if (mlxAReady && tcaSelectChannel(MLX90614_A_CHANNEL))
  {
    myData.ambientTempA = readAmbientTemperature(mlxA);
    myData.objectTempA = readObjectTemperature(mlxA);
  }

  if (mlxBReady && tcaSelectChannel(MLX90614_B_CHANNEL))
  {
    myData.ambientTempB = readAmbientTemperature(mlxB);
    myData.objectTempB = readObjectTemperature(mlxB);
  }

  if (bhAReady && tcaSelectChannel(BH1750_A_CHANNEL))
  {
    myData.lightLuxA = bh1750A.readLightLevel();
    if (myData.lightLuxA < 0)
    {
      myData.lightLuxA = NAN;
    }
  }

  if (bhBReady && tcaSelectChannel(BH1750_B_CHANNEL))
  {
    myData.lightLuxB = bh1750B.readLightLevel();
    if (myData.lightLuxB < 0)
    {
      myData.lightLuxB = NAN;
    }
  }

  tcaDisableChannels();

//   Serial.print(myData.voltage);

  if (isnan(myData.voltageA) || isnan(myData.currentA) ||
      isnan(myData.powerA) || isnan(myData.energyA) ||
      isnan(myData.frequencyA) || isnan(myData.powerFactorA))
  {
    Serial.println("PZEM1 read failed; skipping ESP-NOW send");
    delay(500);
    return;
  }

  Serial.printf("V: %.1f V, I: %.3f A, P: %.1f W, E: %.3f kWh, "
                "F: %.1f Hz, PF: %.2f\n",
                myData.voltageA, myData.currentA, myData.powerA,
                myData.energyA, myData.frequencyA, myData.powerFactorA);

if (isnan(myData.voltageB) || isnan(myData.currentB) || isnan(myData.powerB) ||
    isnan(myData.energyB) || isnan(myData.frequencyB) || isnan(myData.powerFactorB))
{
    Serial.println("PZEM2 Failed to read");
    delay(500);
    return;
}

Serial.printf("V: %.1f V, I: %.3f A, P: %.1f W, E: %.3f kWh, F: %.1f Hz, PF: %.2f\n",
              myData.voltageB, myData.currentB, myData.powerB, myData.energyB, myData.frequencyB, myData.powerFactorB);

  Serial.printf("MLX-A: ambient %.1f C, object %.1f C\n",
                myData.ambientTempA, myData.objectTempA);
  Serial.printf("MLX-B: ambient %.1f C, object %.1f C\n",
                myData.ambientTempB, myData.objectTempB);
  Serial.printf("BH1750-A: %.1f lux\n", myData.lightLuxA);
  Serial.printf("BH1750-B: %.1f lux\n", myData.lightLuxB);




  esp_err_t result = esp_now_send(ESP32S3_MACADDRESS,
                                  reinterpret_cast<const uint8_t *>(&myData),
                                  sizeof(myData));

  if (result == ESP_OK)
  {
    Serial.println("PZEM data queued for sending");
  }
  else
  {
    Serial.printf("ESP-NOW send failed: %d\n", result);
  }

  delay(500);
}