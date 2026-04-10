#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include "MS5837.h"
#include <EEPROM.h>
#include "config.h"
#include <SPI.h>
#include <RH_RF95.h>

// Define RFM69 frequency
#define RF95_FREQ 900.0

//================================================================================================================================================
//                                                              Pin Definitions (XIAO ESP32-C6)

// Direct GPIO pin assignments
const int PIN_ENCODER_A   = D0;   // Encoder A phase
const int PIN_ENCODER_B   = D1;   // Encoder B phase
const int PIN_MOTOR_1     = D2;   // Motor Input 1
const int PIN_MOTOR_2     = D3;   // Motor Input 2
const int PIN_RF95_CS     = D6;   // RFM69 Chip Select
const int PIN_RF95_INT    = D7;   // RFM69 Interrupt
const int PIN_RF95_RST    = D8;   // RFM69 Reset
const int PIN_LIMIT_SW    = D9;   // Limit Switch input
const int PIN_LIMIT_SW_EN = D10;  // Limit Switch Enable

//================================================================================================================================================
//                                                              Global Variables

// Depth control parameters
float current_depth = 0.0;
float target_depth = 0.0;
float depth_tolerance = 0.2;

// Safety limits
const float MAX_DEPTH = 30.0;
const float MIN_DEPTH = 0.0;
const unsigned long MAX_MOTOR_TIME = 120000;

// Encoder tracking (optional, for diagnostics only)
volatile int encoder_count = 0;
volatile int delta = 0;
volatile int a_prev = 0;

// Competition status
bool good_dive = true;
bool dive_again = true;
bool mission_complete = false;

// Pressure sensor object (direct I2C)
MS5837 pressureSensor;

// EEPROM for storing last known depth
const int EEPROM_DEPTH_ADDR = 0;
const int EEPROM_SIZE = 512;

// Telnet server
WiFiServer telnetServer(23);
WiFiClient telnetClient;
bool telnetConnected = false;

// RFM69 Radio
RH_RF95 rf95(PIN_RF95_CS, PIN_RF95_INT);
bool radioAvailable = false;

//================================================================================================================================================
//                                                              Function Prototypes

void piston_out();
void piston_in();
void piston_stop();
float read_depth();
void save_depth();
void load_depth();
bool dive_to_depth(float target_depth_m);
bool surface();
bool hold_depth(float target_depth_m, unsigned long duration_ms);
bool vertical_profile(int profile_num);
void competition_mission();
void handleTelnet();
void printBoth(const String &message);
void printlnBoth(const String &message);
void read_encoder();
void initialize_radio();
void transmitRadioData(const String &data);
void motor_test();

//================================================================================================================================================
//                                                              Setup Function

void setup() {
  // Initialize Serial
  Serial.begin(9600);
  delay(1000);
  Serial.println("\n\n╔════════════════════════════════════════════╗");
  Serial.println("║   NanoFloat Competition Mode - Task 4.1   ║");
  Serial.println("╚════════════════════════════════════════════╝");

  // Configure direct GPIO pins
  pinMode(PIN_ENCODER_A,   INPUT_PULLUP);
  pinMode(PIN_ENCODER_B,   INPUT_PULLUP);
  pinMode(PIN_MOTOR_1,     OUTPUT);
  pinMode(PIN_MOTOR_2,     OUTPUT);
  pinMode(PIN_LIMIT_SW,    INPUT_PULLUP);
  pinMode(PIN_LIMIT_SW_EN, OUTPUT);
  pinMode(PIN_RF95_RST,    OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), updateEncoder, RISING);

  // Enable limit switch
  digitalWrite(PIN_LIMIT_SW_EN, HIGH);

  // Initialize motor to stopped
  piston_stop();

  // Initialize I2C bus
  Wire.begin();

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  load_depth();
  Serial.print("Last recorded depth: ");
  Serial.print(current_depth);
  Serial.println(" m");

  // Initialize pressure sensor
  Serial.println("\nInitializing MS5837 pressure sensor...");
  int attempts = 0;
  while (!pressureSensor.init() && attempts < 10) {
    Serial.println("Pressure sensor init failed! Retrying...");
    delay(2000);
    attempts++;
  }

  if (attempts >= 10) {
    Serial.println("ERROR: Could not initialize pressure sensor!");
    Serial.println("Check connections: White=SDA, Green=SCL");
    while(1) { delay(1000); }
  }

  pressureSensor.setModel(MS5837::MS5837_30BA);  // Bar30 explicit model set
  pressureSensor.setFluidDensity(997);            // Freshwater (use 1029 for seawater)
  Serial.println("✓ Pressure sensor initialized!");

  // Read initial depth
  current_depth = read_depth();
  Serial.print("Current depth: ");
  Serial.print(current_depth);
  Serial.println(" m");

  // Initialize WiFi in Access Point mode
  Serial.println("\nStarting WiFi Access Point...");
  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, 1, 0, 1)) {
    Serial.println("✓ Access Point started successfully");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("✗ Access Point failed to start");
  }

  // Start Telnet server
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.println("\n✓ Telnet server started on port 23");
  Serial.print("Connect via: telnet ");
  Serial.println(WiFi.softAPIP());

  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║          SYSTEM READY FOR MISSION         ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\nMission will auto-start in 10 seconds...");
  Serial.println("(Or call competition_mission() manually)\n");

  // Initialize radio transmitter
  initialize_radio();

  // Optional motor test
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║  Press 'M' within 5 seconds for motor test ║");
  Serial.println("╚════════════════════════════════════════════╝\n");

  unsigned long menuTimeout = millis();
  while (millis() - menuTimeout < 9000) {
    if (Serial.available() > 0) {
      char input = Serial.read();
      if (input == 'M' || input == 'm') {
        motor_test();
        break;
      }
    }
  }
  Serial.println("Skipping motor test - proceeding with mission\n");
}

// //================================================================================================================================================
// //                                                              RFM69 Radio Functions

void initialize_radio() {
  
  pinMode(PIN_RF95_RST, OUTPUT);
  digitalWrite(PIN_RF95_RST, HIGH);

  Serial.println("\nInitializing Nanofloat Radio...");

  // Manual reset
  digitalWrite(PIN_RF95_RST, LOW);
  delay(10);
  digitalWrite(PIN_RF95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("Nanofloat radio init failed");
    radioAvailable = false;
    return;
  }
  Serial.println("Nanofloat radio init OK!");

  if(!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    radioAvailable = false;
    return;
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
  radioAvailable = true;
  Serial.println("Nanofloat radio init OK!");
}

int16_t packetnum = 0; // For counting packets

void transmitRadioData(const String &data) {
  if (!radioAvailable) {
    Serial.println("Nanofloat radio not available");
    return;
  }

  Serial.println("Sending to rf95_server");

  char packetSize[100];
  String radioPacket = "D:" + String(current_depth, 2) +
                        "m | Temp: " + String(pressureSensor.temperature(), 1) +
                        " C | Pressure: " + String(pressureSensor.pressure(), 2) + 
                        " mbar | Encoder: " + String(encoder_count) +
                        " #" + String(packetnum++);
  
  radioPacket.toCharArray(packetSize, sizeof(packetSize));

  Serial.print("Sending ");
  Serial.println(packetSize);
  Serial.println("Sending...");
  delay(10);  
  rf95.send((uint8_t *)packetSize, sizeof(packetSize));
  Serial.println("Waiting for packet to complete..."); 
  delay(10);
  rf95.waitPacketSent();
}

// //================================================================================================================================================
// //                                                              Telnet Helper Functions

void printBoth(const String &message) {
  Serial.print(message);
  if (telnetConnected && telnetClient.connected()) {
    telnetClient.print(message);
  }
}

void printlnBoth(const String &message) {
  Serial.println(message);
  if (telnetConnected && telnetClient.connected()) {
    telnetClient.println(message);
  }
}

void handleTelnet() {
  if (telnetServer.hasClient()) {
    if (telnetClient && telnetClient.connected()) {
      telnetClient.stop();
    }
    telnetClient = telnetServer.available();
    telnetConnected = true;

    telnetClient.println("\n╔════════════════════════════════════════════╗");
    telnetClient.println("║    NanoFloat Telnet Console - Task 4.1    ║");
    telnetClient.println("╚════════════════════════════════════════════╝\n");

    telnetClient.print("Current depth: ");
    telnetClient.print(current_depth, 2);
    telnetClient.println(" m");

    telnetClient.print("Temperature: ");
    telnetClient.print(pressureSensor.temperature(), 1);
    telnetClient.println(" C");

    telnetClient.print("Pressure: ");
    telnetClient.print(pressureSensor.pressure(), 2);
    telnetClient.println(" mbar");

    telnetClient.print("Encoder count: ");
    telnetClient.println(encoder_count);

    telnetClient.print("Mission status: ");
    telnetClient.println(mission_complete ? "COMPLETE" : "IN PROGRESS");

    telnetClient.println("\nMonitoring...\n");
  }

  if (telnetClient && !telnetClient.connected()) {
    telnetClient.stop();
    telnetConnected = false;
    Serial.println("Telnet client disconnected");
  }
}

// //================================================================================================================================================
// //                                                              Encoder Updating

void updateEncoder() {
  // No need for read encoder as iterrupt will handle encoder count automatically.
  if (digitalRead(PIN_ENCODER_A) > digitalRead(PIN_ENCODER_B)) {
    encoder_count++;
  } else {
    encoder_count--;
  }
}

// //================================================================================================================================================
// //                                                              Main Loop

void loop() {
  // Handle telnet connections
  handleTelnet();

  // Auto-start competition mission after 10 seconds
  static bool mission_started = false;
  if (millis() > 10000 && !mission_started && !mission_complete) {
    mission_started = true;
    competition_mission();
  }

  // Continuously monitor depth
  static unsigned long lastRead = 0;
  if (millis() - lastRead > 1000) {
    current_depth = read_depth();

    // Transmit data via radio
    if (current_depth < 0.5) {
      transmitRadioData(" ");
    }

    if (!radioAvailable) {
      printlnBoth("Depth:" + String(current_depth, 2) + " (radio unavailable, switching to Telnet)");
    }

    printBoth("Depth: ");
    printBoth(String(current_depth, 2));
    printBoth(" m | Temp: ");
    printBoth(String(pressureSensor.temperature(), 1));
    printBoth(" C | Pressure: ");
    printBoth(String(pressureSensor.pressure(), 2));
    printBoth(" mbar | Encoder: ");
    printlnBoth(String(encoder_count));

    save_depth();
    lastRead = millis();
  }

  if (digitalRead(PIN_LIMIT_SW) == HIGH) {
    piston_stop();
    printlnBoth("WARNING! Limit switch has been switch off.");
  }

  delay(10);
}

// //================================================================================================================================================
// //                                                              Piston Control Functions

void piston_out() {
  digitalWrite(PIN_MOTOR_1, HIGH);
  digitalWrite(PIN_MOTOR_2, LOW);
}

void piston_in() {
  digitalWrite(PIN_MOTOR_1, LOW);
  digitalWrite(PIN_MOTOR_2, HIGH);
}

void piston_stop() {
  digitalWrite(PIN_MOTOR_1, LOW);
  digitalWrite(PIN_MOTOR_2, LOW);
}

// //================================================================================================================================================
// //                                                              Depth Reading

float read_depth() {
  pressureSensor.read();
  float depth = pressureSensor.depth();

  if (depth < 0) depth = 0;
  if (depth > MAX_DEPTH) depth = MAX_DEPTH;

  return depth;
}

// //================================================================================================================================================
// //                                                              Depth Management

void save_depth() {
  EEPROM.put(EEPROM_DEPTH_ADDR, current_depth);
  EEPROM.commit();
}

void load_depth() {
  EEPROM.get(EEPROM_DEPTH_ADDR, current_depth);
  if (current_depth < 0 || current_depth > MAX_DEPTH) {
    current_depth = 0.0;
  }
}

// //================================================================================================================================================
// //                                                              Depth-Based Movement

bool dive_to_depth(float target_depth_m) {
  printBoth("Diving to ");
  printBoth(String(target_depth_m));
  printlnBoth(" m...");

  if (target_depth_m > MAX_DEPTH) {
    printlnBoth("ERROR: Target depth exceeds maximum safe depth!");
    return false;
  }

  if (target_depth_m < MIN_DEPTH) {
    printlnBoth("ERROR: Target depth below minimum!");
    return false;
  }

  target_depth = target_depth_m;
  unsigned long start_time = millis();

  current_depth = read_depth();

  if (current_depth < target_depth) {
    printlnBoth("Extending piston (diving deeper)...");
    piston_out();

    while (current_depth < (target_depth - depth_tolerance)) {
      current_depth = read_depth();

      if (millis() - start_time > MAX_MOTOR_TIME) {
        piston_stop();
        printlnBoth("ERROR: Motor timeout!");
        return false;
      }

      if (digitalRead(PIN_LIMIT_SW) == HIGH) {
        piston_stop();
        printlnBoth("ERROR: Limit switch hit!");
        return false;
      }

      printBoth("Current depth: ");
      printBoth(String(current_depth, 2));
      printlnBoth(" m");
      delay(500);
    }

  } else if (current_depth > target_depth) {
    printlnBoth("Retracting piston (ascending)...");
    piston_in();

    while (current_depth > (target_depth + depth_tolerance)) {
      current_depth = read_depth();

      if (millis() - start_time > MAX_MOTOR_TIME) {
        piston_stop();
        printlnBoth("ERROR: Motor timeout!");
        return false;
      }

      if (digitalRead(PIN_LIMIT_SW) == HIGH) {
        piston_stop();
        printlnBoth("ERROR: Limit switch hit!");
        return false;
      }

      printBoth("Current depth: ");
      printBoth(String(current_depth, 2));
      printlnBoth(" m");
      delay(500);
    }
  }

  piston_stop();
  current_depth = read_depth();
  printBoth("✓ Reached target! Final depth: ");
  printBoth(String(current_depth, 2));
  printlnBoth(" m");

  return true;
}

bool surface() {
  printlnBoth("Surfacing...");
  return dive_to_depth(0.0);
}

bool hold_depth(float target_depth_m, unsigned long duration_ms) {
  printBoth("Holding depth ");
  printBoth(String(target_depth_m));
  printBoth(" m for ");
  printBoth(String(duration_ms / 1000));
  printlnBoth(" seconds...");

  unsigned long start_time = millis();

  while (millis() - start_time < duration_ms) {
    current_depth = read_depth();

    if (current_depth < (target_depth_m - depth_tolerance)) {
      piston_out();
      delay(200);
      piston_stop();
    } else if (current_depth > (target_depth_m + depth_tolerance)) {
      piston_in();
      delay(200);
      piston_stop();
    }

    printBoth("Holding at ");
    printBoth(String(current_depth, 2));
    printlnBoth(" m");
    delay(1000);
  }

  piston_stop();
  printlnBoth("✓ Hold complete");
  return true;
}

// //================================================================================================================================================
// //                                                              Competition Functions

bool vertical_profile(int profile_num) {
  printlnBoth("");
  printBoth("╔════════════════════════════════════════════╗");
  printlnBoth("");
  printBoth("║       VERTICAL PROFILE ");
  printBoth(String(profile_num));
  printlnBoth("                 ║");
  printBoth("╚════════════════════════════════════════════╝");
  printlnBoth("");

  printlnBoth("\n[Phase 1] Diving to 2.5 meters...");
  if (!dive_to_depth(2.5)) {
    printlnBoth("✗ ERROR: Failed to reach 2.5m!");
    return false;
  }
  printlnBoth("✓ Reached 2.5m");

  printlnBoth("\n[Phase 2] Holding at 2.5 meters for 30 seconds...");
  if (!hold_depth(2.5, 30000)) {
    printlnBoth("✗ ERROR: Failed to hold 2.5m!");
    return false;
  }
  printlnBoth("✓ Held 2.5m for 30 seconds");

  printlnBoth("\n[Phase 3] Ascending to 0.4 meters (40 cm)...");
  if (!dive_to_depth(0.4)) {
    printlnBoth("✗ ERROR: Failed to reach 0.4m!");
    return false;
  }
  printlnBoth("✓ Reached 0.4m");

  printlnBoth("\n[Phase 4] Holding at 0.4 meters for 30 seconds...");
  if (!hold_depth(0.4, 30000)) {
    printlnBoth("✗ ERROR: Failed to hold 0.4m!");
    return false;
  }
  printlnBoth("✓ Held 0.4m for 30 seconds");

  current_depth = read_depth();
  if (current_depth < 0.1) {
    printlnBoth("⚠ WARNING: Float may have broken surface!");
  }

  printlnBoth("");
  printBoth("✓ VERTICAL PROFILE ");
  printBoth(String(profile_num));
  printlnBoth(" COMPLETE");
  printlnBoth("");

  return true;
}

void competition_mission() {
  printlnBoth("\n\n");
  printlnBoth("╔════════════════════════════════════════════╗");
  printlnBoth("║                                            ║");
  printlnBoth("║   STARTING COMPETITION MISSION - TASK 4.1  ║");
  printlnBoth("║                                            ║");
  printlnBoth("╚════════════════════════════════════════════╝");
  printlnBoth("");

  printlnBoth("\n[1/3] Attempting to communicate with station...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttempt = millis();
  bool station_connected = false;

  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    printBoth(".");
    delay(500);
  }
  printlnBoth("");

  if (WiFi.status() == WL_CONNECTED) {
    printlnBoth("✓ Station communication successful!");
    station_connected = true;
  } else {
    printlnBoth("✗ Station communication FAILED!");
    printlnBoth("Aborting mission - cannot proceed without station contact");
    WiFi.mode(WIFI_AP);
    return;
  }

  WiFi.mode(WIFI_AP);
  delay(2000);

  printlnBoth("\n[2/3] Executing Vertical Profile 1...");
  delay(2000);
  bool profile1_success = vertical_profile(1);

  if (profile1_success) {
    printlnBoth("\n✓ VERTICAL PROFILE 1 COMPLETE");
  } else {
    printlnBoth("\n✗ VERTICAL PROFILE 1 FAILED");
  }

  printlnBoth("\nWaiting 10 seconds before Profile 2...");
  delay(10000);

  printlnBoth("\n[3/3] Executing Vertical Profile 2...");
  delay(2000);
  bool profile2_success = vertical_profile(2);

  if (profile2_success) {
    printlnBoth("\n✓ VERTICAL PROFILE 2 COMPLETE");
  } else {
    printlnBoth("\n✗ VERTICAL PROFILE 2 FAILED");
  }

  printlnBoth("\n\n");
  printlnBoth("╔════════════════════════════════════════════╗");
  printlnBoth("║                                            ║");
  printlnBoth("║      COMPETITION MISSION COMPLETE!        ║");
  printlnBoth("║                                            ║");
  printlnBoth("╚════════════════════════════════════════════╝");

  printlnBoth("\n═══════════ MISSION SUMMARY ═══════════");
  printBoth("Station Communication: ");
  printlnBoth(station_connected ? "✓ SUCCESS" : "✗ FAILED");
  printBoth("Vertical Profile 1: ");
  printlnBoth(profile1_success ? "✓ SUCCESS" : "✗ FAILED");
  printBoth("Vertical Profile 2: ");
  printlnBoth(profile2_success ? "✓ SUCCESS" : "✗ FAILED");
  printlnBoth("════════════════════════════════════════");
  printlnBoth("");

  mission_complete = true;
}

//================================================================================================================================================
//                                                                Motor Test

void motor_test() {
    digitalWrite(PIN_MOTOR_1, LOW);
    digitalWrite(PIN_MOTOR_2, LOW);
    
    Serial.println("-------");
    Serial.println("Beginning Motor Test. Input either 1, -1, or 0 to run the motor forwards, backwards, or stop, respectively.");
    Serial.println("Input 'end' to conclude the test.");
    Serial.println("-------");
 
    while (true) {
      if (Serial.available() > 0) {
        String direction = Serial.readStringUntil('\n');
        direction.trim();
        if (direction == "end") {
          digitalWrite(PIN_MOTOR_1, LOW);
          digitalWrite(PIN_MOTOR_2, LOW);
          Serial.println("Motor test concluded");
          break;
        } else if (direction == "1") {
          digitalWrite(PIN_MOTOR_1, HIGH);
          digitalWrite(PIN_MOTOR_2, LOW);
          Serial.println("Running motor forwards");
        } else if (direction == "-1") {
          digitalWrite(PIN_MOTOR_1, LOW);
          digitalWrite(PIN_MOTOR_2, HIGH);
          Serial.println("Running motor backwards");
        } else if (direction == "0") {
          digitalWrite(PIN_MOTOR_1, LOW);
          digitalWrite(PIN_MOTOR_2, LOW);
          Serial.println("Stopping motor");
        } else {
          Serial.println("Input invalid");
        }
      }
    }
}
