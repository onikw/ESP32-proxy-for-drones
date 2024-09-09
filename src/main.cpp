#include <Arduino.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <WiFi.h>

HardwareSerial mySerial(1);

uint8_t system_id = 1;
uint8_t component_id = 200;
uint8_t target_system = 1;
uint8_t target_component = 1;

const char* ssid = "toya89029830";
const char* password = "21378317";

IPAddress local_IP(192, 168, 18, 90);
IPAddress gateway(192, 168, 18, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8); 
IPAddress secondaryDNS(8, 8, 4, 4);

WebServer server(80);

double longitude = 0.0;
double latitude = 0.0;
double altitude = 0.0;
bool dataReceived = false; 

void setup() {
  delay(2000);
  Serial.begin(115200);

  // Ustawienie statycznego IP
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
  }

  Serial.println("\nConnected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Endpoint do ustawiania współrzędnych
server.on("/set-coordinates", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      String body = server.arg("plain");

      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, body);

      if (error) {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
        return;
      }

      longitude = doc["long"];
      latitude = doc["lat"];
      altitude = doc["altitude"];

      // Wysyłanie odpowiedzi do klienta przed zamknięciem serwera
      server.send(200, "application/json", "{\"status\":\"success\"}");
      
      // Wypisywanie otrzymanych danych do konsoli
      Serial.println("Received data:");
      Serial.print("Longitude: ");
      Serial.println(longitude);
      Serial.print("Latitude: ");
      Serial.println(latitude);
      Serial.print("Altitude: ");
      Serial.println(altitude);

      // Opóźnienie, aby klient miał czas na otrzymanie odpowiedzi
      delay(1000); 

      // Zamknięcie serwera i połączenia Wi-Fi
      server.stop();
      WiFi.disconnect(true);
      Serial.println("Server stopped and Wi-Fi disconnected.");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No data received\"}");
    }
});

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();

  // Możesz dodać tutaj dowolne inne operacje, które chcesz wykonać po odebraniu danych.
}
