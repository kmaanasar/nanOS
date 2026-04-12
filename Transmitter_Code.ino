#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <Wire.h>
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
const int PIN_LIMIT_SW    = D7;   // Limit Switch Input 
const int PIN_RF95_SCK    = D8;   // RFM95 Clock
const int PIN_RF95_MISO    = D9;  // RFM95 Serial Out 
const int PIN_RF95_MOSI = D10;  // RFM95 Serial In 

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
bool mission_complete = false;

// Pressure sensor object (direct I2C)
MS5837 pressureSensor;

// EEPROM for storing last known depth
const int EEPROM_DEPTH_ADDR = 0;
const int EEPROM_SIZE = 512;

// Wifi web server
WiFiServer server(80);

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
void update_encoder();
void initialize_radio();
void transmitRadioData();
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
  pinMode(PIN_RF95_RST,    OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), update_encoder, RISING);

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
  Serial.println("Pressure sensor initialized!");

  // Read initial dept
  current_depth = read_depth();
  Serial.print("Current depth: ");
  Serial.print(current_depth);
  Serial.println(" m");

  Serial.println(); 
  Serial.println("Configuring access point..."); 


  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.begin(); 

  Serial.println("\nAP configured successfully!");

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
// //                                                              RFM95 Radio Functions

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

  // if(!rf95.setFrequency(RF95_FREQ)) {
  //   Serial.println("setFrequency failed");
  //   radioAvailable = false;
  //   return;
  // }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
  radioAvailable = true;
  Serial.println("Nanofloat radio init OK!");
}

int16_t packetnum = 0; // For counting packets

void transmitRadioData() {
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
// //                                                              Encoder Updating

void update_encoder() {
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
  // Handle wifi connections
  WiFiClient client = server.available();

  if (client) {
    Serial.println("New web client connected.");
    String currentLine = "";

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (currentLine.length() == 0) {
            // Send HTTP response
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // HTML page with auto-refresh every 2 seconds
            client.println("<!DOCTYPE html><html>");
            client.println("<head>");
            client.println("<meta http-equiv='refresh' content='2'>");
            client.println("<title>NanoFloat Monitor</title>");
            client.println("<style>");
            client.println("body { font-family: monospace; background: #0b1628; color: #dbeafe; padding: 30px; }");
            client.println("h1 { color: #2dd4bf; } .card { background: #1e3a5f; padding: 15px; margin: 10px 0; border-radius: 8px; }");
            client.println(".val { font-size: 1.5em; color: #67e8f9; font-weight: bold; }");
            client.println("</style></head><body>");
            client.println("<h1>NanoFloat 1.2 — Live Monitor</h1>");

            client.println("<div class='card'><p>Depth</p><p class='val'>" + String(current_depth, 2) + " m</p></div>");
            client.println("<div class='card'><p>Temperature</p><p class='val'>" + String(pressureSensor.temperature(), 1) + " °C</p></div>");
            client.println("<div class='card'><p>Pressure</p><p class='val'>" + String(pressureSensor.pressure(), 2) + " mbar</p></div>");
            client.println("<div class='card'><p>Encoder Count</p><p class='val'>" + String(encoder_count) + "</p></div>");
            client.println("<div class='card'><p>Mission Status</p><p class='val'>" + String(mission_complete ? "COMPLETE" : "IN PROGRESS") + "</p></div>");

            client.println("</body></html>");
            client.println();
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }

    client.stop();
    Serial.println("Web client disconnected.");
  }
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
      transmitRadioData();
    }

    if (!radioAvailable) {
      Serial.println("Depth:" + String(current_depth, 2) + " (radio unavailable, switching to Wifi)");
    }

    Serial.println("Depth: ");
    Serial.println(String(current_depth, 2));
    Serial.print(" m | Temp: ");
    Serial.println(String(pressureSensor.temperature(), 1));
    Serial.print(" C | Pressure: ");
    Serial.println(String(pressureSensor.pressure(), 2));
    Serial.print(" mbar | Encoder: ");
    Serial.println(String(encoder_count));

    save_depth();
    lastRead = millis();
  }

  if (digitalRead(PIN_LIMIT_SW) == HIGH) {
    piston_stop();
    Serial.println("WARNING! Limit switch has been switch off.");
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
  Serial.println("Diving to ");
  Serial.println(String(target_depth_m));
  Serial.println(" m...");

  if (target_depth_m > MAX_DEPTH) {
    Serial.println("ERROR: Target depth exceeds maximum safe depth!");
    return false;
  }

  if (target_depth_m < MIN_DEPTH) {
    Serial.println("ERROR: Target depth below minimum!");
    return false;
  }

  target_depth = target_depth_m;
  unsigned long start_time = millis();

  current_depth = read_depth();

  if (current_depth < target_depth) {
    Serial.println("Extending piston (diving deeper)...");
    piston_out();

    while (current_depth < (target_depth - depth_tolerance)) {
      current_depth = read_depth();

      if (millis() - start_time > MAX_MOTOR_TIME) {
        piston_stop();
        Serial.println("ERROR: Motor timeout!");
        return false;
      }

      if (digitalRead(PIN_LIMIT_SW) == HIGH) {
        piston_stop();
        Serial.println("ERROR: Limit switch hit!");
        return false;
      }

      Serial.print("Current depth: ");
      Serial.println(String(current_depth, 2));
      Serial.println(" m");
      delay(500);
    }

  } else if (current_depth > target_depth) {
    Serial.println("Retracting piston (ascending)...");
    piston_in();

    while (current_depth > (target_depth + depth_tolerance)) {
      current_depth = read_depth();

      if (millis() - start_time > MAX_MOTOR_TIME) {
        piston_stop();
        Serial.println("ERROR: Motor timeout!");
        return false;
      }

      if (digitalRead(PIN_LIMIT_SW) == HIGH) {
        piston_stop();
        Serial.println("ERROR: Limit switch hit!");
        return false;
      }

      Serial.print("Current depth: ");
      Serial.println(String(current_depth, 2));
      Serial.println(" m");
      delay(500);
    }
  }

  piston_stop();
  current_depth = read_depth();
  Serial.print("Reached target! Final depth: ");
  Serial.println(String(current_depth, 2));
  Serial.println(" m");

  return true;
}

bool surface() {
  Serial.println("Surfacing...");
  return dive_to_depth(0.0);
}

bool hold_depth(float target_depth_m, unsigned long duration_ms) {
  Serial.print("Holding depth ");
  Serial.print(String(target_depth_m));
  Serial.print(" m for ");
  Serial.print(String(duration_ms / 1000));
  Serial.println(" seconds...");

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

    Serial.print("Holding at ");
    Serial.print(String(current_depth, 2));
    Serial.println(" m");
    delay(1000);
  }

  piston_stop();
  Serial.println("Hold complete");
  return true;
}

// //================================================================================================================================================
// //                                                              Competition Functions

bool vertical_profile(int profile_num) {
  Serial.println("");
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("");
  Serial.print("║       VERTICAL PROFILE ");
  Serial.print(String(profile_num));
  Serial.println("                 ║");
  Serial.println("╚═════════════════════════════════════════╗");
  Serial.println("");

  Serial.println("\n[Phase 1] Diving to 2.5 meters...");
  if (!dive_to_depth(2.5)) {
    Serial.println("ERROR: Failed to reach 2.5m!");
    return false;
  }
  Serial.println("Reached 2.5m");

  Serial.println("\n[Phase 2] Holding at 2.5 meters for 30 seconds...");
  if (!hold_depth(2.5, 30000)) {
    Serial.println("ERROR: Failed to hold 2.5m!");
    return false;
  }
  Serial.println("Held 2.5m for 30 seconds");

  Serial.println("\n[Phase 3] Ascending to 0.4 meters (40 cm)...");
  if (!dive_to_depth(0.4)) {
    Serial.println("ERROR: Failed to reach 0.4m!");
    return false;
  }
  Serial.println("Reached 0.4m");

  Serial.println("\n[Phase 4] Holding at 0.4 meters for 30 seconds...");
  if (!hold_depth(0.4, 30000)) {
    Serial.println("ERROR: Failed to hold 0.4m!");
    return false;
  }
  Serial.println("Held 0.4m for 30 seconds");

  current_depth = read_depth();
  if (current_depth < 0.1) {
    Serial.println("WARNING: Float may have broken surface!");
  }

  Serial.println("");
  Serial.print("VERTICAL PROFILE ");
  Serial.print(String(profile_num));
  Serial.println(" COMPLETE");
  Serial.println("");

  return true;
}

void competition_mission() {
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║                                            ║");
  Serial.println("║   STARTING COMPETITION MISSION - TASK 4.1  ║");
  Serial.println("║                                            ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("");

  Serial.println("\n Executing Vertical Profiles");
  delay(2000);
  bool profile1_success = vertical_profile(1);

  if (profile1_success) {
    Serial.println("\n VERTICAL PROFILE 1 COMPLETE");
  } else {
    Serial.println("\n VERTICAL PROFILE 1 FAILED");
  }

  Serial.println("\nWaiting 10 seconds before Profile 2...");
  delay(10000);

  Serial.println("\n[3/3] Executing Vertical Profile 2...");
  delay(2000);
  bool profile2_success = vertical_profile(2);

  if (profile2_success) {
    Serial.println("\n VERTICAL PROFILE 2 COMPLETE");
  } else {
    Serial.println("\n VERTICAL PROFILE 2 FAILED");
  }

  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║                                            ║");
  Serial.println("║      COMPETITION MISSION COMPLETE!        ║");
  Serial.println("║                                            ║");
  Serial.println("╚════════════════════════════════════════════╝");

  Serial.println("\n═══════════ MISSION SUMMARY ═══════════");
  Serial.print("Vertical Profile 1: ");
  Serial.println(profile1_success ? "SUCCESS" : "FAILED");
  Serial.print("Vertical Profile 2: ");
  Serial.println(profile2_success ? "SUCCESS" : "FAILED");
  Serial.println("════════════════════════════════════════");
  Serial.println("");

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
