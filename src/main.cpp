#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <EEPROM.h>
#include <set>
#include <vector>
#include <random>
#include "esp_task_wdt.h"
#include "ntc_polimaster.h"

const uint kSerialBaudRate = 115200;

const char* kBaseSsid = "POLIMASTER"; // AP SSID
char ap_ssid[32]; // Buffer for SSID
const char* kApPassword = ""; // AP password
const int kEepromSize = 512; // EEPROM size
const int kMaxEntries = 10; // Max SSID and password entries

AsyncWebServer server(80); // Web server instance

// Pin configurations
const int kFanPin = 27;
const int kFanPWMChannel = 0;
const int kHeaterPin = 26;
const int kHeaterPWMChannel = 1;
const int kUltrasonicPin = 25;
const int kUltrasonicEnablePin = 33;
const int kNtcPin = 32; // Temperature sensor pin

// Thermistor configuration
const double kNtcRefVoltage = 3.325;
const double kNtcVoltageNominalFactor = 1.0146;
const double kNtcSerialResistor = 15000;
const double kNtcResistanceDeltaRatio = 0.85;

// Device state variables
int fanSpeed = 0;
int heaterPower = 0;
int ultrasonicVoltage = 0;
float temperature = 0.0;
bool ultrasonicEnabled = false;
bool scanning = false;
std::vector<String> ssids;

String currentSSID;
String currentSSIDPassword;
String ssid, password;
bool connecting = false;
unsigned long timer; // Declare timer here

// Function prototypes
double readNTCTemperature();
void startScan();
void handleScanResults();
void onTimer();
bool connectToWiFi(int numWiFiFound);
void setupServerRoutes();
void saveWiFiCredentials(const char* ssid, const char* password, bool success);
String readStringFromEEPROM(int address);
void writeStringToEEPROM(int address, const char* str);
void setupWiFi();
void setupMDNS();
void setupSPIFFS();
void setupPWM();

// Function prototypes
bool tryConnect(const String& ssid, const String& password);
void handlePWMRequest(AsyncWebServerRequest *request, int &powerVariable, int pin, int minValue, int maxValue);
void handleUltrasonicVoltage(AsyncWebServerRequest *request);
void handleUltrasonicSwitch(AsyncWebServerRequest *request);
String getSettingsJSON();
String getSSIDsJSON();
void setupEditRoutes();
String getEditSSIDsJSON();


void setup() {
  Serial.begin(kSerialBaudRate);
  EEPROM.begin(kEepromSize);

  setupPWM();
  setupWiFi();
  setupMDNS();
  setupSPIFFS();
  setupServerRoutes();
  
  server.begin();
}

void setupPWM() {
  ledcSetup(kFanPWMChannel, 5000, 8);
  ledcAttachPin(kFanPin, 0);
  ledcSetup(kHeaterPWMChannel, 5000, 8);
  ledcAttachPin(kHeaterPin, 1);
  pinMode(kUltrasonicEnablePin, OUTPUT);
  digitalWrite(kUltrasonicEnablePin, HIGH);

  if (kUltrasonicPin == 25 || kUltrasonicPin == 26) {
    analogWriteResolution(8);
    dacWrite(kUltrasonicPin, 255);
  } else {
    Serial.println("Invalid DAC pin for ultrasonic.");
  }
}

void setupWiFi() {
  WiFi.mode(WIFI_AP_STA);

  // Check for nearby networks
  Serial.println("Scanning WiFi networks...");
  int numNetworks = WiFi.scanNetworks();
  bool ssidConflict = false;

  // Check if the base SSID is in use
  for (int i = 0; i < numNetworks; i++) {
    if (WiFi.SSID(i) == kBaseSsid) {
      ssidConflict = true;
      break;
    }
  }

  // If there's a conflict, append a random postfix
  if (ssidConflict) {
    snprintf(ap_ssid, sizeof(ap_ssid), "%s_%d", kBaseSsid, random(1000, 9999));
  } else {
    snprintf(ap_ssid, sizeof(ap_ssid), "%s", kBaseSsid);
  }

  // Start the AP
  WiFi.softAP(ap_ssid, kApPassword);
  Serial.println("AP Started with SSID: " + String(ap_ssid));

  if (!connectToWiFi(numNetworks)) {
    Serial.println("No valid WiFi credentials found.");
  }

  Serial.print("STA IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());
}

void setupMDNS() {
  if (!MDNS.begin("polimaster")) {
    Serial.println("Error starting mDNS");
  }
  Serial.println("mDNS responder started");
}

void setupSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error occurred while mounting SPIFFS");
  }
}


bool connectToWiFi(int numWiFiFound) {
  Serial.println("Scanning WiFi networks...");
  int n = numWiFiFound;
  Serial.println("Available networks:");
  std::set<String> uniqueSSIDs;

  for (int j = 0; j < n; ++j) {
    uniqueSSIDs.insert(WiFi.SSID(j));
  }

  for (const auto& ssid: uniqueSSIDs){
    Serial.println(ssid);
  }

  for (int i = 0; i < kMaxEntries; i++) {
    char validByte = EEPROM.read(i * 33 + 32); // Read validity first

    // Only read SSID and password if the entry is valid
    if (validByte == 1) {
      String savedSSID = readStringFromEEPROM(i * 33); // First 16 bytes
      String password = readStringFromEEPROM(i * 33 + 16); // Next 16 bytes

      if (uniqueSSIDs.count(savedSSID) && !password.isEmpty() && tryConnect(savedSSID, password)) {        
        return true; // Successful connection
      }
    }
  }
  return false; // No connection made
}

bool tryConnect(const String& ssid, const String& password) {
  Serial.print("Trying WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid.c_str(), password.c_str());

  int currentTry = 0;
  while (WiFi.status() != WL_CONNECTED && currentTry < 3) {
        // Reset the watchdog timer
        ++currentTry;
        esp_task_wdt_reset();
        delay(1000); // Delay for a bit before retrying
  }

  if(currentTry < 3){
    currentSSID = ssid;
    currentSSIDPassword = password;
    Serial.println("WIFI connected: "+ssid);
    return true;
  }
  else{
    WiFi.disconnect();
    Serial.println("WIFI connection failed: "+ssid);
    return false;   
  }
}

void saveWiFiCredentials(const char* ssid, const char* password, bool success) {
  for (int i = kMaxEntries - 1; i > 0; i--) {
    writeStringToEEPROM(i * 33, readStringFromEEPROM((i - 1) * 33).c_str());
    writeStringToEEPROM(i * 33 + 16, readStringFromEEPROM((i - 1) * 33 + 16).c_str());
    EEPROM.write(i * 33 + 32, EEPROM.read((i - 1) * 33 + 32));
  }
  
  writeStringToEEPROM(0, ssid);
  writeStringToEEPROM(16, password);
  EEPROM.write(32, success ? 1 : 0);
  EEPROM.commit();
}

void writeStringToEEPROM(int address, const char* str) {
  for (int i = 0; i < 16; i++) {
    EEPROM.write(address + i, str[i] ? str[i] : 0);
  }
}

String readStringFromEEPROM(int address) {
  String data = "";
  for (int i = 0; i < 16; i++) {
    char c = EEPROM.read(address + i);
    if (c == 0) break;
    data += c;
  }
  return data;
}

void setupServerRoutes() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false);
  });

  server.on("/setFanSpeed", HTTP_GET, [](AsyncWebServerRequest *request){
    handlePWMRequest(request, fanSpeed, kFanPin, 0, 100);
  });

  server.on("/setHeaterPower", HTTP_GET, [](AsyncWebServerRequest *request){
    handlePWMRequest(request, heaterPower, kHeaterPin, 20, 95);
  });

  server.on("/setUltrasonicVoltage", HTTP_GET, [](AsyncWebServerRequest *request){
    handleUltrasonicVoltage(request);
  });

  server.on("/setUltrasonicEnabled", HTTP_GET, [](AsyncWebServerRequest *request){
    handleUltrasonicSwitch(request);
  });

  server.on("/getTemperature", HTTP_GET, [](AsyncWebServerRequest *request){
    temperature = readNTCTemperature();
    request->send(200, "text/plain", String(temperature));
  });

  server.on("/getSettings", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", getSettingsJSON());
  });

  server.on("/scanSSIDs", HTTP_GET, [](AsyncWebServerRequest *request) {
    startScan();
    request->send(200, "text/plain", "Scanning started. Check back later for results.");
  });

  server.on("/getSSIDs", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!scanning && !ssids.empty()) {
      request->send(200, "application/json", getSSIDsJSON());
    } else {
      request->send(200, "text/plain", "Scan in progress or no results available.");
    }
  });

  server.on("/connectWiFi", HTTP_GET, [](AsyncWebServerRequest *request) {    
    if (request->hasParam("ssid") && request->hasParam("password")) {
      // Access parameters safely
      ssid = request->getParam("ssid")->value();
      password = request->getParam("password")->value();
    } else {
      request->send(400, "text/plain", "Missing parameters.");
      return;
    }
    
    request->send(200, "text/plain", "ssid and password received.");    
    // Start the non-blocking delay
    connecting = true;
    // Set a timer for 100 ms (using millis() in loop)
    // Note: If using a timer library, you might replace this with that logic
    timer = millis() + 100; // Adjust based on your needs
  });

  // Additional static file serving routes
  server.serveStatic("/qrcode.min.js", SPIFFS, "/qrcode.min.js");
  server.serveStatic("/eeprom_edit.html", SPIFFS, "/eeprom_edit.html");

  setupEditRoutes();
}

void setupEditRoutes() {
  server.on("/getEditSSIDs", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", getEditSSIDsJSON());
  });

  server.on("/editSSID", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("index") && request->hasParam("ssid") && request->hasParam("password")) {
      int index = request->getParam("index")->value().toInt();
      String ssid = request->getParam("ssid")->value();
      String password = request->getParam("password")->value();

      if (index >= 0 && index < kMaxEntries) {
        writeStringToEEPROM(index * 33, ssid.c_str());
        writeStringToEEPROM(index * 33 + 16, password.c_str());
        EEPROM.write(index * 33 + 32, 1);
        EEPROM.commit();
        request->send(200, "text/plain", "SSID and password updated successfully.");
      } else {
        request->send(400, "text/plain", "Index out of range.");
      }
    } else {
      request->send(400, "text/plain", "Missing parameters.");
    }
  });

  server.on("/deleteSSID", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("index")) {
      int index = request->getParam("index")->value().toInt();
      if (index >= 0 && index < kMaxEntries) {
        writeStringToEEPROM(index * 33, "");
        writeStringToEEPROM(index * 33 + 16, "");
        EEPROM.write(index * 33 + 32, 0);
        EEPROM.commit();
        request->send(200, "text/plain", "SSID deleted successfully.");
      } else {
        request->send(400, "text/plain", "Index out of range.");
      }
    } else {
      request->send(400, "text/plain", "Missing parameters.");
    }
  });

  server.on("/pasteSSID", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("targetIndex") && request->hasParam("sourceIndex")) {
      int targetIndex = request->getParam("targetIndex")->value().toInt();
      int sourceIndex = request->getParam("sourceIndex")->value().toInt();

      if (targetIndex >= 0 && targetIndex < kMaxEntries && sourceIndex >= 0 && sourceIndex < kMaxEntries) {
        String ssid = readStringFromEEPROM(sourceIndex * 33);
        String password = readStringFromEEPROM(sourceIndex * 33 + 16);
        writeStringToEEPROM(targetIndex * 33, ssid.c_str());
        writeStringToEEPROM(targetIndex * 33 + 16, password.c_str());
        EEPROM.write(targetIndex * 33 + 32, 1);
        EEPROM.commit();
        request->send(200, "text/plain", "Pasted SSID and password.");
      } else {
        request->send(400, "text/plain", "Index out of range.");
      }
    } else {
      request->send(400, "text/plain", "Missing parameters.");
    }
  });
}

void handlePWMRequest(AsyncWebServerRequest *request, int &powerVariable, int pin, int minValue, int maxValue) {
  if (request->hasParam("value")) {
    powerVariable = request->getParam("value")->value().toInt();

    int write_channel;
    switch (pin)
    {
    case kFanPin:
      write_channel = kFanPWMChannel;
      break;
    case kHeaterPin:
      write_channel = kHeaterPWMChannel;    
    default:
      break;
    }

    ledcWrite(write_channel, map(powerVariable, minValue, maxValue, 0, 255));
    request->send(200, "text/plain", "Value set to " + String(powerVariable));
  } else {
    request->send(400, "text/plain", "Missing value parameter");
  }
}

void handleUltrasonicVoltage(AsyncWebServerRequest *request) {
  if (request->hasParam("value")) {
    ultrasonicVoltage = request->getParam("value")->value().toInt();
    dacWrite(kUltrasonicPin, map(ultrasonicVoltage, 1.6, 24, 255, 112));
    request->send(200, "text/plain", "Ultrasonic ntc_voltage set to " + String(ultrasonicVoltage));
  } else {
    request->send(400, "text/plain", "Missing value parameter");
  }
}

void handleUltrasonicSwitch(AsyncWebServerRequest *request) {
  if (request->hasParam("value")) {
    ultrasonicEnabled = request->getParam("value")->value() == "true";
    digitalWrite(kUltrasonicEnablePin, ultrasonicEnabled ? LOW : HIGH);
    request->send(200, "text/plain", "Ultrasonic enabled set to " + String(ultrasonicEnabled ? "true" : "false"));
  } else {
    request->send(400, "text/plain", "Missing value parameter");
  }
}

String getSettingsJSON() {
  return "{\"fanSpeed\": " + String(fanSpeed) +
         ", \"heaterPower\": " + String(heaterPower) +
         ", \"ultrasonicVoltage\": " + String(ultrasonicVoltage) +
         ", \"ultrasonicSwitch\": " + (ultrasonicEnabled ? "true" : "false") +
         ", \"ipAddress\": \"" + WiFi.localIP().toString() +
         "\", \"currentSSID\": \"" + String(WiFi.SSID()) + "\"}";
}

String getSSIDsJSON() {
  String json = "[";
  for (size_t i = 0; i < ssids.size(); i++) {
    json += "\"" + ssids[i] + "\"";
    if (i < ssids.size() - 1) {
      json += ",";
    }
  }
  json += "]";
  return json;
}

String getEditSSIDsJSON() {
  ssids.clear();
  for (int i = 0; i < kMaxEntries; i++) {
    String savedSSID = readStringFromEEPROM(i * 33);
    ssids.push_back(savedSSID);
  }
  return getSSIDsJSON();
}

void loop() {
  handleScanResults();
  // Check if it's time to connect
  if (connecting && millis() >= timer) {
    onTimer();
  }
}

double readNTCTemperature() {  
  // Read the analog value from the NTC pin
  int analogValue = analogRead(kNtcPin);

  // Convert the analog reading (0-4095) to a voltage
  double voltage = (analogValue / 4095.0) * kNtcRefVoltage;
  double nominal_voltage = kNtcVoltageNominalFactor * voltage;

  // Calculate the resistance of the NTC thermistor
  // Using the voltage divider formula: R_ntc = R_serial * (V_test / V_ntc - 1)   
  if (nominal_voltage <= 2.78  ){
    nominal_voltage += 0.133 * (2.78 - nominal_voltage)/2.78;
  }

  double ntcResistance = nominal_voltage * kNtcSerialResistor / (kNtcRefVoltage - nominal_voltage);

  // Serial.println("pin_adc_value: "+String(analogValue));
  // Serial.println("ntcResistance: "+String(ntcResistance));
  // Serial.println("nominal_voltage: "+String(nominal_voltage));
  // Serial.println("temperature: "+String(tmp));
  // Serial.println("");

  return resistanceToTemperature(ntcResistance);
}

void startScan() {
  if (!scanning) {
    scanning = true;
    ssids.clear();
    WiFi.scanNetworks(true); // Start non-blocking scan
  }
}

void handleScanResults() {
  int numNetworks = WiFi.scanComplete();
  if (numNetworks >= 0) {
    std::set<String> uniqueSSIDs;

    for (int i = 0; i < numNetworks; i++) {
      uniqueSSIDs.insert(WiFi.SSID(i));
    }

    ssids.clear();
    for (const auto& ssid : uniqueSSIDs) {
      ssids.push_back(ssid);
    }

    WiFi.scanDelete(); // Clean up after scan
    scanning = false; // Reset scanning flag
  }
}

void onTimer() {
  if (connecting) {
    // Call your Wi-Fi connection function
    
    if (tryConnect(ssid, password)) {
      saveWiFiCredentials(ssid.c_str(), password.c_str(), true);
      // Optionally send a success message or redirect
      } else {
        // Retry with current credentials or handle failure
      tryConnect(currentSSID, currentSSIDPassword);
    }

    connecting = false; // Reset the state
  }
}