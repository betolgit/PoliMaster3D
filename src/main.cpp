#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>

// WiFi配置
const char* sta_ssid = "廖氏锁业";         // 现有的 Wi-Fi 网络 SSID
const char* sta_password = "1234qwer";     // 现有的 Wi-Fi 网络密码
const char* ap_ssid = "newPeterDoor";      // ESP32 作为 AP 时的 SSID
const char* ap_password = "";   // ESP32 作为 AP 时的密码（可选）

const u32_t port = 80;
// const char* ssid = "newPeterDoor"; // 新的SSID
// const char* password = ""; // 空密码

// Web服务器实例
AsyncWebServer server(port);

// PWM引脚配置
const int fanPin = 27;
const int heaterPin = 26;
const int ultrasonicPin = 25;
const int ultrasonicEnablePin = 33;

// 温度传感器引脚配置
const int ntcPin = 34;

// 温度和PWM设置
int fanSpeed = 0; // 风扇转速PWM值
int heaterPower = 0; // 加热块PWM值
int ultrasonicVoltage = 0; // 超声波电压（DAC值）
float temperature = 0.0; // 温度值
bool ultrasonicEnabled = false; // 超声波开关

// 温度传感器读取函数声明
float readNTCTemperature();

void setup() {
  Serial.begin(115200);

  ledcSetup(0, 5000, 8); // 风扇PWM通道
  ledcAttachPin(fanPin, 0);
  ledcSetup(1, 5000, 8); // 加热块PWM通道
  ledcAttachPin(heaterPin, 1);

  // 将 ultrasonicPin 配置为 DAC
  // 假设 ultrasonicPin 是 GPIO 25 或 GPIO 26
  // ESP32 的 DAC 通道分别是 25 (DAC1) 和 26 (DAC2)
  if (ultrasonicPin == 25 || ultrasonicPin == 26) {
    // 设置 ultrasonicPin 为 DAC 输出
    analogWriteResolution(8); // 8-bit 分辨率
    dacWrite(ultrasonicPin, 255); // 初始化 DAC 输出为 0
  } else {
    // 处理其他引脚的配置（如果需要）
    Serial.println("ultrasonicPin 不是有效的 DAC 引脚。");
  }

  // 设置 ESP32 为 STA 和 AP 模式
  WiFi.mode(WIFI_AP_STA);
  
  
  // 配置 AP 模式
  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("AP Started");
  
  // 配置 STA 模式
  WiFi.begin(sta_ssid, sta_password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("Connected!");
  
  // 打印 STA 模式的 IP 地址
  Serial.print("STA IP Address: ");
  IPAddress sta_ip = WiFi.localIP();
  Serial.println(sta_ip.toString());

  // 打印 AP 模式的 IP 地址
  Serial.print("AP IP Address: ");
  IPAddress ap_ip = WiFi.softAPIP();
  Serial.println(ap_ip.toString());

  // Serial.print("Connecting to WiFi");
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("Connected!");
  // Serial.print("IP Address: ");
  // IPAddress ip = WiFi.localIP();
  // Serial.println(ip.toString() + ":" + String(port));



  // WiFi.mode(WIFI_AP);
  // WiFi.softAP(ssid, password); // 设置SSID和密码（空密码）

  // Serial.println("WiFi Access Point Started");
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.softAPIP()); // 打印AP模式下的IP地址

  //mDNS
  if (!MDNS.begin("polimaster")) {  // "esp32"是你为设备设置的mDNS主机名
    Serial.println("Error starting mDNS");
    return;
  }
  Serial.println("mDNS responder started");
  Serial.print("You can access the web server at http://polimaster.local:"+String(port)+"/");

  if (!SPIFFS.begin()) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false);
  });

  server.on("/setFanSpeed", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      fanSpeed = request->getParam("value")->value().toInt();
      ledcWrite(0, map(fanSpeed, 0, 100, 0, 255));
      request->send(200, "text/plain", "Fan speed set to " + String(fanSpeed));
    } else {
      request->send(400, "text/plain", "Missing value parameter");
    }
  });

  server.on("/setHeaterPower", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      heaterPower = request->getParam("value")->value().toInt();
      ledcWrite(1, map(heaterPower, 20, 95, 0, 255));
      request->send(200, "text/plain", "Heater power set to " + String(heaterPower));
    } else {
      request->send(400, "text/plain", "Missing value parameter");
    }
  });

  server.on("/setUltrasonicVoltage", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      ultrasonicVoltage = request->getParam("value")->value().toInt();
      dacWrite(ultrasonicPin, map(ultrasonicVoltage, 1.6, 24, 255, 112));
      request->send(200, "text/plain", "Ultrasonic voltage set to " + String(ultrasonicVoltage));
    } else {
      request->send(400, "text/plain", "Missing value parameter");
    }
  });

    // 新增处理超声波开关状态的路由
  server.on("/setUltrasonicEnabled", HTTP_GET, [](AsyncWebServerRequest *request){
      if (request->hasParam("value")) {
          ultrasonicEnabled = request->getParam("value")->value() == "true"; // 更新全局变量
          // 在此处更新 ultrasonicEnabled 的状态，可能需要保存到 EEPROM 或其他存储
          // 例如: eepromWriteUltrasonicEnabled(ultrasonicEnabled);

          request->send(200, "text/plain", "Ultrasonic enabled set to " + String(ultrasonicEnabled ? "true" : "false"));
      } else {
          request->send(400, "text/plain", "Missing value parameter");
      }
  });

  server.on("/getTemperature", HTTP_GET, [](AsyncWebServerRequest *request){
    temperature = readNTCTemperature();
    request->send(200, "text/plain", String(temperature));
  });

  server.on("/getSettings", HTTP_GET, [](AsyncWebServerRequest *request){
      String json = "{\"fanSpeed\": " + String(fanSpeed) + 
                    ", \"heaterPower\": " + String(heaterPower) + 
                    ", \"ultrasonicVoltage\": " + String(ultrasonicVoltage) + 
                    ", \"ultrasonicSwitch\": " + (ultrasonicEnabled ? "true" : "false") + "}";
      request->send(200, "application/json", json);
  });

  server.begin();
}

void loop() {
  // Main loop does nothing, as the server handles requests asynchronously
}

// 读取NTC温度的函数实现
float readNTCTemperature() {
  int rawValue = analogRead(ntcPin);
  float resistance = (1023.0 / rawValue - 1) * 10000;
  float temperature = 1 / (log(resistance / 10000) / 3950 + 1 / 298.15) - 273.15;
  return temperature;
}
