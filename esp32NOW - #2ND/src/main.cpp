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

#define TCA_addr 0x70

BH1750 LightSensorA(0x23);
BH1750 LightSensorB(0x23);

Adafruit_MLX90614 TempSensorA;
Adafruit_MLX90614 TempSensorB;


constexpr uint8_t RECEIVER_CHANNEL = 1;

uint8_t ESP32S3_MACADDRESS[] = {0x90, 0xE5, 0xB1, 0xD1, 0x62, 0x58};

PZEM004Tv30 pzem(Serial1, PZEM_RX_PIN, PZEM_TX_PIN);
PZEM004Tv30 pzem2(Serial2, PZEM2_rx, PZEM2_tx);


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

void tcaSelect(uint8_t channel)
{
  if (channel > 7)
  {
    return;
  }
  
  Wire.beginTransmission(TCA_addr);
  Wire.write(1 << channel);
  Wire.endTransmission();

}

void tcaScanSetup()
{
  Serial.println("Scanning i2c's mux channels...");
  for (uint8_t ch = 0; ch < 8; ch++)
  {
    tcaSelect(ch);
    Serial.print("Channel: ");
    Serial.println(ch);

    for (uint8_t addr = 1; addr < 128; addr++)
    {
      if (addr == TCA_addr)
      {
        continue;
      }

      Wire.beginTransmission(addr);

      if (Wire.endTransmission() == 0)
      {
        Serial.print("Sensor Found at 0x");
        Serial.println(addr, HEX);
      }
      
      
    }
    
  }


  tcaSelect(7);
  if (LightSensorA.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
  {
    Serial.println("LuxSensorA Ready");
  }
  else
  {
    Serial.println("LuxSensorA Error");
  }

  tcaSelect(6);
  if (LightSensorB.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
  {
    Serial.println("LuxSensorB Ready");
  }
  else
  {
    Serial.println("LuxSensorB Error");
  }
  
  
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

  // PZEM-004T modules need a short startup period after ESP32 power-on.
  Serial1.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
  Serial2.begin(9600, SERIAL_8N1, PZEM2_rx, PZEM2_tx);
  delay(2000);

  Serial1.flush();
  Serial2.flush();

  tcaScanSetup();

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

  tcaSelect(7);
  tcaSelect(6);
  tcaSelect(5);
  tcaSelect(4);

  myData.lightLuxA = LightSensorA.readLightLevel();
  myData.lightLuxB = LightSensorB.readLightLevel();
  myData.ambientTempA = TempSensorA.readAmbientTempC();
  myData.ambientTempB = TempSensorB.readAmbientTempC();




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
  // 



  if (isnan(myData.voltageB) || isnan(myData.currentB) || isnan(myData.powerB) ||
    isnan(myData.energyB) || isnan(myData.frequencyB) || isnan(myData.powerFactorB))
  {
    Serial.println("PZEM2 Failed to read");
    delay(500);
    return;
  }

  Serial.printf("V: %.1f V, I: %.3f A, P: %.1f W, E: %.3f kWh, F: %.1f Hz, PF: %.2f\n", 
    myData.voltageB, myData.currentB, myData.powerB, myData.energyB, myData.frequencyB, myData.powerFactorB);
  // 



  if (isnan(myData.lightLuxA) || isnan(myData.lightLuxB))
  {
    Serial.println("BH1750 FAILED TO READ!");
    delay(500);
    return;
  }
  Serial.printf("LUX A : %.1flux, LUX B : %.1flux\n", myData.lightLuxA, myData.lightLuxB);






  if (isnan(myData.ambientTempA) || isnan(myData.ambientTempB))
  {
    Serial.println("MLX FAILED! TO READ");
    delay(500);
    return;
  }
  Serial.printf("TEMPERATURE A : %.2f C, TEMPERATURE B : %.1f C", myData.ambientTempA, myData.ambientTempB);
  



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