#include <Arduino.h>
#include <HardwareSerial.h>
#include <MAVLink.h>

// Używamy Serial1 na ESP32-C3
HardwareSerial mySerial(1);

void setup() {
  // Inicjalizacja portu szeregowego dla debugowania
  Serial.begin(115200);
  // Inicjalizacja portu szeregowego do komunikacji z FC
  mySerial.begin(57600, SERIAL_8N1, 2, 3);
  Serial.println("Połączono z FC przez port szeregowy z użyciem MAVLink");
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