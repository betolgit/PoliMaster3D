// #include <WiFi.h>
// #include <ESPAsyncWebServer.h>
// #include <AsyncTCP.h>

// // WiFi����
// const char* ssid = "PeterDoor";
// const char* password = "1234qwer";

// // Web������ʵ��
// AsyncWebServer server(80);

// // PWM��������
// const int fanPin = 25; // ����PWM��������
// const int heaterPin = 26; // ���ȿ�PWM��������
// const int ultrasonicPin = 27; // ��������ѹ��������

// // �¶ȴ�������������
// const int ntcPin = 34; // NTC�¶ȴ���������

// // �¶Ⱥ�PWM����
// int fanSpeed = 0; // ����ת��PWMֵ
// int heaterPower = 0; // ���ȿ�PWMֵ
// int ultrasonicVoltage = 0; // ��������ѹ��DACֵ��
// float temperature = 0.0; // �¶�ֵ

// // �¶ȴ�������ȡ��������
// float readNTCTemperature();

// void setup() {
//   Serial.begin(115200);

//   ledcSetup(0, 5000, 8); // ����PWMͨ��
//   ledcAttachPin(fanPin, 0);
//   ledcSetup(1, 5000, 8); // ���ȿ�PWMͨ��
//   ledcAttachPin(heaterPin, 1);
//   ledcSetup(2, 5000, 8); // ��������ѹ����ͨ��
//   ledcAttachPin(ultrasonicPin, 2);

//   WiFi.begin(ssid, password);
//   Serial.print("Connecting to WiFi");
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("Connected!");
//   Serial.print("IP Address: ");
//   Serial.println(WiFi.localIP());

//   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
//     String html = "<html><body>";
//     html += "<h1>ESP32 Control</h1>";
//     html += "<p>����ת�� (%): <input type='range' id='fanSpeed' min='0' max='100' step='1' onchange='updateFanSpeed()' value='" + String(fanSpeed) + "'> <span id='fanSpeedValue'>" + String(fanSpeed) + "</span></p>";
//     html += "<p>�������¶� (��C): <input type='range' id='heaterPower' min='20' max='95' step='1' onchange='updateHeaterPower()' value='" + String(heaterPower) + "'> <span id='heaterPowerValue'>" + String(heaterPower) + "</span></p>";
//     html += "<p>������������ѹ (V): <input type='range' id='ultrasonicVoltage' min='10' max='24' step='0.1' onchange='updateUltrasonicVoltage()' value='" + String(ultrasonicVoltage / 255.0 * 14.0 + 10.0) + "'> <span id='ultrasonicVoltageValue'>" + String(ultrasonicVoltage / 255.0 * 14.0 + 10.0) + "</span></p>";
//     html += "<p>������ʵʱ�¶�: <span id='temperature'>" + String(temperature) + "</span> ��C</p>";
//     html += "<script>";
//     html += "function updateFanSpeed() { var fanSpeed = document.getElementById('fanSpeed').value; document.getElementById('fanSpeedValue').innerText = fanSpeed; fetch('/setFanSpeed?value=' + fanSpeed); }";
//     html += "function updateHeaterPower() { var heaterPower = document.getElementById('heaterPower').value; document.getElementById('heaterPowerValue').innerText = heaterPower; fetch('/setHeaterPower?value=' + heaterPower); }";
//     html += "function updateUltrasonicVoltage() { var ultrasonicVoltage = document.getElementById('ultrasonicVoltage').value; document.getElementById('ultrasonicVoltageValue').innerText = ultrasonicVoltage; fetch('/setUltrasonicVoltage?value=' + ultrasonicVoltage); }";
//     html += "setInterval(function() { fetch('/getTemperature').then(response => response.text()).then(data => document.getElementById('temperature').innerText = data); }, 1000);";
//     html += "</script>";
//     html += "</body></html>";
    
//     request->send(200, "text/html", html);
//   });

//   server.on("/setFanSpeed", HTTP_GET, [](AsyncWebServerRequest *request){
//     if (request->hasParam("value")) {
//       fanSpeed = request->getParam("value")->value().toInt();
//       ledcWrite(0, map(fanSpeed, 0, 100, 0, 255));
//       request->send(200, "text/plain", "Fan speed set to " + String(fanSpeed));
//     } else {
//       request->send(400, "text/plain", "Missing value parameter");
//     }
//   });

//   server.on("/setHeaterPower", HTTP_GET, [](AsyncWebServerRequest *request){
//     if (request->hasParam("value")) {
//       heaterPower = request->getParam("value")->value().toInt();
//       ledcWrite(1, map(heaterPower, 20, 95, 0, 255));
//       request->send(200, "text/plain", "Heater power set to " + String(heaterPower));
//     } else {
//       request->send(400, "text/plain", "Missing value parameter");
//     }
//   });

//   server.on("/setUltrasonicVoltage", HTTP_GET, [](AsyncWebServerRequest *request){
//     if (request->hasParam("value")) {
//       float voltage = request->getParam("value")->value().toFloat();
//       ultrasonicVoltage = map(voltage, 10, 24, 0, 255);
//       ledcWrite(2, ultrasonicVoltage);
//       request->send(200, "text/plain", "Ultrasonic voltage set to " + String(voltage));
//     } else {
//       request->send(400, "text/plain", "Missing value parameter");
//     }
//   });

//   server.on("/getTemperature", HTTP_GET, [](AsyncWebServerRequest *request){
//     temperature = readNTCTemperature();
//     request->send(200, "text/plain", String(temperature));
//   });
  
//   server.begin();
// }

// void loop() {
//   temperature = readNTCTemperature();
// }

// float readNTCTemperature() {
//   int adcValue = analogRead(ntcPin);
//   float voltage = adcValue * (3.3 / 4095.0);
//   float resistance = (3.3 - voltage) * 10000 / voltage;
//   float temperature = (1.0 / (log(resistance / 10000.0) / 3435 + 1 / 298.15)) - 273.15;
//   return temperature;
// }
