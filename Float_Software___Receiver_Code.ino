#include <Arduino.h>
#include <SPI.h>
#include <RH_RF69.h>

// Define pin assignments for RFM69
const int RFM69_CS  = D6;   // RFM69 Chip Select
const int RFM69_RST = D7;   // RFM69 Reset
const int RFM69_INT = D8;   // RFM69 Interrupt

#define RF69_FREQ 900.0     // Must match transmitter frequency

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// LED for indication
const int LED = 13;

void setup() {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("RFM69 Receiver!");

    pinMode(LED, OUTPUT);
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, HIGH);

    // Manual reset
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    digitalWrite(RFM69_RST, HIGH);
    delay(10);

    // Initialize RFM69
    if (!rf69.init()) {
        Serial.println("RFM69 radio init failed");
        while (1);
    }
    Serial.println("RFM69 radio init OK!");

    // Set frequency
    if (!rf69.setFrequency(RF69_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
    }
    Serial.print("Set Frequency to: ");
    Serial.println(RF69_FREQ);

    // Match transmitter power setting
    rf69.setTxPower(20, true);
}

void loop() {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf69.available()) {
        if (rf69.recv(buf, &len)) {
            digitalWrite(LED, HIGH);
            Serial.print("Received: ");
            Serial.println((char*)buf);
            Serial.print("RSSI: ");
            Serial.println(rf69.lastRssi(), DEC);
            digitalWrite(LED, LOW);
        } else {
            Serial.println("Receive failed");
        }
    }

    delay(100);
}