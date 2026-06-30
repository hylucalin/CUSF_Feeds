#include <Arduino.h>

HardwareSerial Radio(1);

// Choose safe, free GPIOs on ESP32-S3
#define RADIO_RX 18   // ESP32 RX  <- CC2530 TX
#define RADIO_TX 17   // ESP32 TX  -> CC2530 RX

void setup() {
  Serial.begin(115200);      // USB debug output
  Radio.begin(
    115200,                 // MUST match CC2530 baud rate
    SERIAL_8N1,
    RADIO_RX,
    RADIO_TX
  );

  Serial.println("CC2530 Receiver Ready");
}

void loop() {
  while (Radio.available()) {
    uint8_t c = Radio.read();
    Serial.write(c);        // forward received data to PC
  }
}
