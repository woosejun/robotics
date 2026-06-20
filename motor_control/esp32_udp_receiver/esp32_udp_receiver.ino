#include <Arduino.h>

const unsigned long SERIAL_BAUD = 115200;
const unsigned long COMMAND_TIMEOUT_MS = 500;

char packetBuffer[128];
size_t packetLength = 0;

unsigned long lastCommandMs = 0;
unsigned long lastStatusMs = 0;

uint8_t checksum(const char* text) {
  uint8_t value = 0;

  while (*text) {
    value ^= (uint8_t)(*text);
    text++;
  }

  return value;
}

int hexValue(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }

  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }

  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }

  return -1;
}

bool parsePacket(char* packet, int& sequence, String& robotState, int& errorX) {
  if (packet[0] != '$') {
    return false;
  }

  char* star = strchr(packet, '*');

  if (star == NULL || strlen(star) < 3) {
    return false;
  }

  int high = hexValue(star[1]);
  int low = hexValue(star[2]);

  if (high < 0 || low < 0) {
    return false;
  }

  *star = '\0';

  char* body = packet + 1;
  uint8_t expected = (uint8_t)((high << 4) | low);

  if (checksum(body) != expected) {
    return false;
  }

  char* seqText = strtok(body, ",");
  char* stateText = strtok(NULL, ",");
  char* errorText = strtok(NULL, ",");

  if (seqText == NULL || stateText == NULL || errorText == NULL) {
    return false;
  }

  sequence = atoi(seqText);
  robotState = String(stateText);
  errorX = atoi(errorText);

  return true;
}

void stopMotors() {
  // Motor code comes later. For now, this firmware verifies UART commands.
}

void applyRobotCommand(int sequence, const String& robotState, int errorX) {
  Serial.print("RX seq=");
  Serial.print(sequence);
  Serial.print(" state=");
  Serial.print(robotState);
  Serial.print(" errorX=");
  Serial.println(errorX);

  Serial.print("OK ");
  Serial.println(sequence);
}

void handleLine(char* line) {
  Serial.print("UART raw: ");
  Serial.println(line);

  int sequence = 0;
  String robotState;
  int errorX = 0;

  if (parsePacket(line, sequence, robotState, errorX)) {
    lastCommandMs = millis();
    applyRobotCommand(sequence, robotState, errorX);
  } else {
    Serial.println("Bad packet");
    Serial.println("ERR");
  }
}

void readJetsonSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      packetBuffer[packetLength] = '\0';

      if (packetLength > 0) {
        handleLine(packetBuffer);
      }

      packetLength = 0;
      return;
    }

    if (packetLength < sizeof(packetBuffer) - 1) {
      packetBuffer[packetLength] = c;
      packetLength++;
    } else {
      packetLength = 0;
      Serial.println("UART packet too long");
      Serial.println("ERR too long");
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println("ESP32 UART receiver start");
  Serial.println("Using ESP32 UART header RX/TX");
  Serial.println("Connect Jetson TX to ESP32 RX, Jetson RX to ESP32 TX, GND to GND");

  stopMotors();
}

void loop() {
  readJetsonSerial();

  if (lastCommandMs > 0 && millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    stopMotors();
  }

  if (millis() - lastStatusMs > 2000) {
    lastStatusMs = millis();
    Serial.println("UART waiting");
  }
}
