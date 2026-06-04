/*
  ADS_pressure_example.ino

  ESP32-S3 + ADS1115 pressure sensor voltage-divider example.

  Hardware assumptions:
    - ADS1115 ADDR tied to GND, so I2C address is 0x48.
    - ADS1115 VDD/VIN powered from ESP32-S3 3V3.
    - ADS1115 GND tied to ESP32-S3 GND and pressure sensor GND.
    - ADS1115 SDA -> ESP32-S3 GPIO1.
    - ADS1115 SCL -> ESP32-S3 GPIO2.
    - Pressure sensor green output is divided down before entering ADS1115.
    - Divider midpoint connected to ADS1115 A3 and/or A1.

  Arduino library required:
    - Adafruit ADS1X15
*/

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define I2C_SDA_PIN 1
#define I2C_SCL_PIN 2
#define ADS1115_ADDR 0x48

// ADS1115 analogue input channels in use.
#define ADC_CH_PRESSURE_A3 3   // ADS1115 A3
#define ADC_CH_PRESSURE_A1 1   // ADS1115 A1

// Measured resistor values.
// Circuit: sensor signal -> R_TOP -> ADS input node -> R_BOTTOM -> GND
const float R_TOP_OHMS_A3    = 4300.0f;
const float R_BOTTOM_OHMS_A3 = 8160.0f;

const float R_TOP_OHMS_A1    = 4310.0f;
const float R_BOTTOM_OHMS_A1 = 8300.0f;

const float DIVIDER_RATIO_A3 = R_BOTTOM_OHMS_A3 / (R_TOP_OHMS_A3 + R_BOTTOM_OHMS_A3);
const float DIVIDER_RATIO_A1 = R_BOTTOM_OHMS_A1 / (R_TOP_OHMS_A1 + R_BOTTOM_OHMS_A1);

// Typical 0.5-4.5 V pressure transducer mapping.
// Change these if your sensor is different.
const float SENSOR_MIN_V = 0.5f;
const float SENSOR_MAX_V = 4.5f;
const float SENSOR_FULL_SCALE_PSI = 150.0f;

Adafruit_ADS1115 ads;
bool ads_ok = false;
unsigned long last_retry_ms = 0;

bool initialiseADS1115() {
  Serial.println("Initialising ADS1115...");

  // GAIN_ONE gives +/-4.096 V full-scale.
  // With ADS1115 powered from 3.3 V, keep the physical input below 3.3 V.
  ads.setGain(GAIN_ONE);

  if (!ads.begin(ADS1115_ADDR, &Wire)) {
    Serial.println("ADS1115 not found at 0x48. Check VDD, GND, SDA, SCL, ADDR.");
    return false;
  }

  Serial.println("ADS1115 found at 0x48.");
  return true;
}

float readAdcVoltage(uint8_t channel) {
  int16_t raw = ads.readADC_SingleEnded(channel);
  return ads.computeVolts(raw);
}

float dividerToSensorVoltage(float adc_voltage, float divider_ratio) {
  return adc_voltage / divider_ratio;
}

float sensorVoltageToPsi(float sensor_voltage) {
  float psi = (sensor_voltage - SENSOR_MIN_V) *
              SENSOR_FULL_SCALE_PSI /
              (SENSOR_MAX_V - SENSOR_MIN_V);

  // Small negative values are normal near zero because of sensor offset/noise.
  if (psi < 0.0f) psi = 0.0f;
  return psi;
}

void printChannel(uint8_t channel, const char *name, float divider_ratio) {
  float adc_v = readAdcVoltage(channel);
  float sensor_v = dividerToSensorVoltage(adc_v, divider_ratio);
  float psi = sensorVoltageToPsi(sensor_v);
  float bar = psi * 0.0689476f;

  Serial.print(name);
  Serial.print(" | ratio: ");
  Serial.print(divider_ratio, 6);
  Serial.print(" | ADS input: ");
  Serial.print(adc_v, 4);
  Serial.print(" V | estimated sensor output: ");
  Serial.print(sensor_v, 4);
  Serial.print(" V | pressure: ");
  Serial.print(psi, 2);
  Serial.print(" psi / ");
  Serial.print(bar, 3);
  Serial.println(" bar");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32-S3 ADS1115 pressure example");
  Serial.println("---------------------------------");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);  // 100 kHz is conservative and robust for wiring on perfboard.

  Serial.print("I2C SDA GPIO: ");
  Serial.println(I2C_SDA_PIN);
  Serial.print("I2C SCL GPIO: ");
  Serial.println(I2C_SCL_PIN);

  Serial.print("A3 divider ratio: ");
  Serial.println(DIVIDER_RATIO_A3, 6);
  Serial.print("A1 divider ratio: ");
  Serial.println(DIVIDER_RATIO_A1, 6);

  Serial.print("A3 voltage at ADS if sensor output is 5.0 V: ");
  Serial.print(5.0f * DIVIDER_RATIO_A3, 3);
  Serial.println(" V");
  Serial.print("A1 voltage at ADS if sensor output is 5.0 V: ");
  Serial.print(5.0f * DIVIDER_RATIO_A1, 3);
  Serial.println(" V");

  ads_ok = initialiseADS1115();
}

void loop() {
  if (!ads_ok) {
    // Keep retrying instead of crashing/stopping.
    if (millis() - last_retry_ms > 2000) {
      last_retry_ms = millis();
      ads_ok = initialiseADS1115();
    }
    delay(50);
    return;
  }

  printChannel(ADC_CH_PRESSURE_A3, "A3", DIVIDER_RATIO_A3);
  printChannel(ADC_CH_PRESSURE_A1, "A1", DIVIDER_RATIO_A1);
  Serial.println();

  delay(500);
}
