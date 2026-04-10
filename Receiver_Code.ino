#include <Arduino.h>
#include <SPI.h>
#include <RH_RF69.h>

// Define pin assignments for RFM95
const int RFM95_CS  = D6;   // RFM95 Chip Select
const int RFM95_RST = D7;   // RFM95 Reset
const int RFM95_INT = D8;   // RFM95 Interrupt

#define RF95_FREQ 900.0     // Must match transmitter frequency

// Singleton instance of the radio driver
RH_RF95 rf69(RFM95_CS, RFM95_INT);

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("NanoFloat reciver!");

  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("Nanofloat reciever init failed");
    while (1);
  }
  Serial.println("Nanofloat reciever init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
}

void loop() {
  if (rf95.available()){
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    } else {
      Serial.println("Receive failed");
    }
  }
}
