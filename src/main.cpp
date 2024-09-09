#include <Arduino.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <MAVLink.h>
#include <sbus.h>

void sendMission(float lat_wp, float lon_wp, float altitude, float delay_seconds);
boolean fetch_and_read(bfs::SbusRx& sbus_rx);
void send(bfs::SbusTx& sbus_tx);

uint8_t system_id = 1;
uint8_t component_id = MAV_COMP_ID_AUTOPILOT1;
uint8_t target_system = 1;       // ID Twojego drona
uint8_t target_component = MAV_COMP_ID_AUTOPILOT1;

HardwareSerial mySerial(1);  // Używamy portu szeregowego 1

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
double delay_seconds = 300.0;

bool dataReceived = false; 
bool setMavlink = false; 
bool set_proxy = false;

// dane do sbus
const int RX_PIN = 4;
const int TX_PIN = 5;
const int SW_PIN = 20;

// throttle
const int THR_CH = 2;
const int THR_0 = 200;
const int THR_60 = 1200;

// arm
const int ARM_CH = 4;
const int ARM_FALSE = 200;
const int ARM_TRUE = 1800;

// mode
const int MODE_CH = 5;
const int MODE_1 = 200;
const int MODE_2 = 1000;
const int MODE_3 = 1800;

// czas spadku [ms]
const int TIME_FALL = 500;
const int TIME_60 = 4500;
const int REPEAT = 20;

int thr_val = THR_0;
int arm_val = ARM_FALSE;
int mode_val = MODE_1;

bfs::SbusData data;
boolean is_mounted = false;
boolean is_released = true;
boolean is_auto = false;

void setup() {
  delay(2000);
  Serial.begin(115200);

  // Konfiguracja Wi-Fi
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

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
      delay_seconds = doc["delay"];

      server.send(200, "application/json", "{\"status\":\"success\"}");
      dataReceived = true;

      // Zatrzymanie serwera po odebraniu danych
      server.stop();
      WiFi.disconnect(true);
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No data received\"}");
    }
  });
  server.begin();
}

void loop() {
  server.handleClient();

  if (dataReceived) {
    if (!setMavlink) {
      mySerial.begin(57600, SERIAL_8N1, 2, 3);  // Komunikacja MAVLink na pinach 2 i 3
      setMavlink = true;
      sendMission(latitude, longitude, altitude, delay_seconds);
    }

    while (mySerial.available()) {
      mavlink_message_t msg;
      mavlink_status_t status;
      uint8_t c = mySerial.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        Serial.println("Odebrano wiadomość MAVLink");
        mySerial.end();  // Zakończenie komunikacji MAVLink
        Serial.println("Zakończono komunikację MAVLink");

        // Przełączenie na SBUS po zakończeniu komunikacji MAVLink
        set_proxy = true;
        dataReceived = false;
        break;
      }
    }

    if (set_proxy) {
      // Inicjalizacja SBUS na tych samych pinach co wcześniej MAVLink
      mySerial.begin(100000, SERIAL_8E2, RX_PIN, TX_PIN);  // Przełączenie na piny SBUS
      bfs::SbusRx sbus_rx(&mySerial, RX_PIN, TX_PIN, true);
      bfs::SbusTx sbus_tx(&mySerial, RX_PIN, TX_PIN, true);
      
      // Logika SBUS
      if (is_released && digitalRead(SW_PIN) == LOW) {
        is_mounted = true;
        is_released = false;
      }
      if (!is_released && digitalRead(SW_PIN) == HIGH) {
        is_auto = true;
        is_mounted = false;
        thr_val = THR_0;
        arm_val = ARM_TRUE;
        mode_val = MODE_2;

        for (int i = 0; i < TIME_FALL; i += REPEAT) {
          fetch_and_read(sbus_rx);
          send(sbus_tx);
          delay(REPEAT);
        }

        thr_val = THR_60;
        for (int i = 0; i < TIME_60; i += REPEAT) {
          fetch_and_read(sbus_rx);
          send(sbus_tx);
          delay(REPEAT);
        }

        fetch_and_read(sbus_rx);
        mode_val = MODE_3;
        send(sbus_tx);

        is_released = true;
      }
      if (fetch_and_read(sbus_rx)) {
        send(sbus_tx);
      }
    }
  }
}

boolean fetch_and_read(bfs::SbusRx& sbus_rx) {
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    if (!is_mounted && data.ch[ARM_CH] >= ARM_TRUE - 100) {
      is_auto = false;
    }
    return true;
  }
  return false;
}

void send(bfs::SbusTx& sbus_tx) {
  if (is_auto) {
    if (!is_released) {
      data.ch[THR_CH] = thr_val;
    }
    data.ch[ARM_CH] = arm_val;
    data.ch[MODE_CH] = mode_val;
  }
  sbus_tx.data(data);
  sbus_tx.Write();
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
