#include <Arduino.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <WiFi.h>


#include <HardwareSerial.h>
#include <MAVLink.h>


void sendMission(float lat_wp, float lon_wp, float altitude, float delay_seconds);
uint8_t system_id = 1;
uint8_t component_id = MAV_COMP_ID_AUTOPILOT1;
uint8_t target_system = 1;       // ID Twojego drona
uint8_t target_component = MAV_COMP_ID_AUTOPILOT1;

HardwareSerial mySerial(1);

// uint8_t system_id = 1;
// uint8_t component_id = 200;
// uint8_t target_system = 1;
// uint8_t target_component = 1;

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

bool setMavlink = false; 

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
      dataReceived = true;

      
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No data received\"}");
    }
});

  server.begin();
  Serial.println("HTTP server started");
}


void loop() {
  mavlink_message_t msg;
  mavlink_status_t status;

  // Obsługa klienta
  server.handleClient();

  // Sprawdzanie, czy dane zostały odebrane
  if (dataReceived) {
    
    // Inicjalizacja MAVLink na pierwsze uruchomienie
    if (setMavlink == false) {
      mySerial.begin(57600, SERIAL_8N1, 2, 3);  // Inicjalizacja komunikacji szeregowej
      Serial.println("Połączono z FC przez port szeregowy z użyciem MAVLink");
      setMavlink = true;
      sendMission(float(latitude), float(longitude), float(altitude), 10.0);
    }

    while (mySerial.available()) {
      uint8_t c = mySerial.read();  // Odczyt bajtu danych
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        Serial.println("Odebrano wiadomość MAVLink");
      }
    }
  }
}


void sendMission(float lat_wp, float lon_wp, float altitude, float delay_seconds) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;

  // 1. Wysłanie wiadomości MISSION_COUNT z liczbą punktów w misji
  uint16_t mission_count = 4; // Start + waypoint + delay + lądowanie
  mavlink_msg_mission_count_pack(
    system_id,
    component_id,
    &msg,
    target_system,
    target_component,
    mission_count,
    0, // target_component (plan)
    0  // mission_type (standard mission)
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
  Serial.println("MISSION_COUNT wysłano");

  delay(1000); // Poczekaj chwilę, aby kontroler lotu przetworzył wiadomość

  // 2. Wysłanie punktu startowego (pozycja 0)
  mavlink_msg_mission_item_int_pack(
    system_id,
    component_id,
    &msg,
    target_system,
    target_component,
    0, // Seq
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Frame
    MAV_CMD_NAV_WAYPOINT, // Command
    1, // Current (1 oznacza, że to pierwszy punkt)
    1, // Autocontinue
    0.0, // Param1
    0.0, // Param2
    0.0, // Param3
    0.0, // Param4
    (int32_t)(lat_wp * 1e7), // X (szerokość geograficzna)
    (int32_t)(lon_wp * 1e7), // Y (długość geograficzna)
    altitude, // Z (wysokość w metrach)
    MAV_MISSION_TYPE_MISSION // mission_type
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
  Serial.println("Punkt startowy wysłano");

  delay(1000);

  // 3. Wysłanie waypointa (pozycja 1)
  mavlink_msg_mission_item_int_pack(
    system_id,
    component_id,
    &msg,
    target_system,
    target_component,
    1, // Seq
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Frame
    MAV_CMD_NAV_WAYPOINT, // Command
    0, // Current
    1, // Autocontinue
    0.0, // Param1
    0.0, // Param2
    0.0, // Param3
    0.0, // Param4
    (int32_t)(lat_wp * 1e7), // X
    (int32_t)(lon_wp * 1e7), // Y
    altitude, // Z
    MAV_MISSION_TYPE_MISSION // mission_type
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
  Serial.println("Waypoint wysłano");

  delay(1000);

  // 4. Wysłanie komendy opóźnienia (Delay) (pozycja 2)
  mavlink_msg_mission_item_int_pack(
    system_id,
    component_id,
    &msg,
    target_system,
    target_component,
    2, // Seq
    MAV_FRAME_MISSION, // Frame
    MAV_CMD_CONDITION_DELAY, // Command
    0, // Current
    1, // Autocontinue
    delay_seconds, // Param1 (czas opóźnienia w sekundach)
    0.0, // Param2
    0.0, // Param3
    0.0, // Param4
    0, // X (nieużywane)
    0, // Y (nieużywane)
    0.0, // Z (nieużywane)
    MAV_MISSION_TYPE_MISSION // mission_type
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
  Serial.println("Opóźnienie wysłano");

  delay(1000);

  // 5. Wysłanie punktu lądowania (pozycja 3)
  mavlink_msg_mission_item_int_pack(
    system_id,
    component_id,
    &msg,
    target_system,
    target_component,
    3, // Seq
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Frame
    MAV_CMD_NAV_LAND, // Command
    0, // Current
    1, // Autocontinue
    0.0, // Param1
    0.0, // Param2
    0.0, // Param3
    0.0, // Param4
    (int32_t)(lat_wp * 1e7), // X
    (int32_t)(lon_wp * 1e7), // Y
    0.0, // Z (0 dla lądowania)
    MAV_MISSION_TYPE_MISSION // mission_type
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
  Serial.println("Lądowanie wysłano");

  delay(1000);
}