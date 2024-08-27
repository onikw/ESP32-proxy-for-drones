#include <Arduino.h>
#include <HardwareSerial.h>
#include <MAVLink.h>

// Używamy Serial1 na ESP32-C3
HardwareSerial mySerial(1);

uint8_t system_id = 1;        // ID systemu ESP32-C3 (ground station)
uint8_t component_id = 200;   // ID komponentu (ground station)
uint8_t target_system = 1;    // ID systemu docelowego (drona)
uint8_t target_component = 1; // ID komponentu docelowego (autopilot)


void sendMission() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // 1. Wysłanie wiadomości MISSION_COUNT z liczbą punktów w misji
  uint16_t mission_count = 2; // Jeden punkt + lądowanie
  mavlink_msg_mission_count_pack(system_id, component_id, &msg, target_system, target_component, mission_count, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
  Serial.println("MISSION_COUNT wysłano");

  delay(1000); // Poczekaj chwilę, aby kontroler lotu przetworzył wiadomość

  // 2. Wysłanie punktu nawigacyjnego (Waypoint)
  float lat = 47.397742;  // Szerokość geograficzna celu (w stopniach)
  float lon = 8.545594;   // Długość geograficzna celu (w stopniach)
  float alt = 10.0;       // Wysokość celu (w metrach)

  mavlink_msg_mission_item_int_pack(
    system_id,                 // ID systemu
    component_id,              // ID komponentu
    &msg,                      // Wskaźnik na wiadomość mavlink
    target_system,             // ID systemu docelowego (dron)
    target_component,          // ID komponentu docelowego (autopilot)
    0,                         // Seq - numer sekwencji punktu misji
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Frame - współrzędne globalne z relatywną wysokością
    MAV_CMD_NAV_WAYPOINT,      // Command - komenda nawigacji do punktu
    0,                         // Current - 0: nie aktywne, 1: aktywny punkt misji
    1,                         // Autocontinue - 1: kontynuacja automatyczna, 0: nie
    0.0,                       // Param1 - np. akceptacja promienia (można ustawić na 0)
    0.0,                       // Param2 - np. czas oczekiwania (można ustawić na 0)
    0.0,                       // Param3 - np. promień przejścia (można ustawić na 0)
    0.0,                       // Param4 - kierunek (można ustawić na 0)
    lat * 1e7,                 // X (szerokość geograficzna * 1e7)
    lon * 1e7,                 // Y (długość geograficzna * 1e7)
    alt * 1000,                // Z (wysokość w mm)
    MAV_MISSION_TYPE_MISSION   // Typ misji
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
  Serial.println("WAYPOINT wysłano");

  delay(1000);

  // 3. Wysłanie komendy lądowania
  mavlink_msg_mission_item_int_pack(
    system_id,                 // ID systemu
    component_id,              // ID komponentu
    &msg,                      // Wskaźnik na wiadomość mavlink
    target_system,             // ID systemu docelowego (dron)
    target_component,          // ID komponentu docelowego (autopilot)
    1,                         // Seq - numer sekwencji punktu misji
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Frame - współrzędne globalne z relatywną wysokością
    MAV_CMD_NAV_LAND,          // Command - komenda lądowania
    0,                         // Current - 0: nie aktywne, 1: aktywny punkt misji
    1,                         // Autocontinue - 1: kontynuacja automatyczna, 0: nie
    0.0,                       // Param1 (niezdefiniowane dla lądowania)
    0.0,                       // Param2 (niezdefiniowane dla lądowania)
    0.0,                       // Param3 (niezdefiniowane dla lądowania)
    0.0,                       // Param4 - kierunek podejścia (można ustawić na 0)
    lat * 1e7,                 // X (szerokość geograficzna * 1e7)
    lon * 1e7,                 // Y (długość geograficzna * 1e7)
    0,                         // Z (wysokość w mm, 0 dla lądowania)
    MAV_MISSION_TYPE_MISSION   // Typ misji
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);
  Serial.println("LAND wysłano");

  delay(1000);
}


void setup() {
  // Inicjalizacja portu szeregowego dla debugowania
  Serial.begin(115200);
  // Inicjalizacja portu szeregowego do komunikacji z FC
  mySerial.begin(57600, SERIAL_8N1, 2, 3);
  Serial.println("Połączono z FC przez port szeregowy z użyciem MAVLink");

  sendMission();
}

void loop() {
  mavlink_message_t msg;
  mavlink_status_t status;

  // Odczytywanie danych z FC
  while (mySerial.available()) {
    uint8_t c = mySerial.read();

    // Dekodowanie wiadomości MAVLink
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Rozpoznano pełną wiadomość MAVLink
      Serial.println("Odebrano wiadomość MAVLink");

      // Przykład: Jeśli wiadomość to HEARTBEAT
      if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        Serial.println("HEARTBEAT odebrany");
      }
    }
  }

  // Przykład: Wysyłanie prostego komunikatu MAVLink - HEARTBEAT
  mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mySerial.write(buf, len);

  delay(1000); // Czekaj 1 sekundę
}