// #include <WiFi.h>
// #include <ESPAsyncWebServer.h>  // 需要安装 ESPAsyncWebServer 库

// const char* ssid = "PeterDoor";
// const char* password = "1234qwer";

// const int output26 = 26;
// const int output27 = 27;

// AsyncWebServer server(80);

// // HTML 页面模板
// String htmlPage = R"rawliteral(
// <!DOCTYPE html>
// <html>
// <head>
// <meta name="viewport" content="width=device-width, initial-scale=1">
// <link rel="icon" href="data:,">
// <style>
// html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center; }
// .button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer; }
// .button2 { background-color: #555555; }
// </style>
// </head>
// <body>
// <h1>ESP32 Web Server</h1>
// <p>GPIO 26 - State %26State%</p>
// <p><a href="/26/%26Action%"><button class="button">%26Button%</button></a></p>
// <p>GPIO 27 - State %27State%</p>
// <p><a href="/27/%27Action%"><button class="button">%27Button%</button></a></p>
// </body>
// </html>
// )rawliteral";

// void handleRoot(AsyncWebServerRequest *request) {
//   String response = htmlPage;
//   String output26State = digitalRead(output26) == HIGH ? "on" : "off";
//   String output27State = digitalRead(output27) == HIGH ? "on" : "off";
//   String action26 = output26State == "off" ? "on" : "off";
//   String action27 = output27State == "off" ? "on" : "off";
//   String button26 = output26State == "off" ? "ON" : "OFF";
//   String button27 = output27State == "off" ? "ON" : "OFF";

//   response.replace("%26State%", output26State);
//   response.replace("%27State%", output27State);
//   response.replace("%26Action%", action26);
//   response.replace("%27Action%", action27);
//   response.replace("%26Button%", button26);
//   response.replace("%27Button%", button27);

//   request->send(200, "text/html", response);
// }

// void setup() {
//   Serial.begin(115200);
//   pinMode(output26, OUTPUT);
//   pinMode(output27, OUTPUT);
//   digitalWrite(output26, LOW);
//   digitalWrite(output27, LOW);

//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("\nWiFi connected.");
//   Serial.println("IP address: " + WiFi.localIP().toString());

//   server.on("/", HTTP_GET, handleRoot);

//   server.on("/26/on", HTTP_GET, [](AsyncWebServerRequest *request){
//     digitalWrite(output26, HIGH);
//     request->redirect("/");
//   });

//   server.on("/26/off", HTTP_GET, [](AsyncWebServerRequest *request){
//     digitalWrite(output26, LOW);
//     request->redirect("/");
//   });

//   server.on("/27/on", HTTP_GET, [](AsyncWebServerRequest *request){
//     digitalWrite(output27, HIGH);
//     request->redirect("/");
//   });

//   server.on("/27/off", HTTP_GET, [](AsyncWebServerRequest *request){
//     digitalWrite(output27, LOW);
//     request->redirect("/");
//   });

//   server.begin();
// }

// void loop() {
//   // 无需手动处理客户端连接
// }
