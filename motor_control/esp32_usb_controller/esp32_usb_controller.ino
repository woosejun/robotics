#include <Arduino.h>

// AceBot car shield (74HC595)
const uint8_t DATA_PIN = 5;
const uint8_t STCP_PIN = 17;
const uint8_t SHCP_PIN = 18;
const uint8_t EN_PIN = 16;

// Each motor uses one 74HC595 bit for forward and one for reverse.
// Channel order: M1=front-left, M2=rear-left,
//                M3=front-right, M4=rear-right.
const uint8_t M1_FORWARD = 0x01;
const uint8_t M1_REVERSE = 0x02;
const uint8_t M2_FORWARD = 0x04;
const uint8_t M2_REVERSE = 0x08;
const uint8_t M3_FORWARD = 0x20;
const uint8_t M3_REVERSE = 0x10;
const uint8_t M4_FORWARD = 0x80;
const uint8_t M4_REVERSE = 0x40;

const uint8_t MOTOR_STOP = 0;
const uint8_t MOTOR_FORWARD =
  M1_FORWARD | M2_FORWARD | M3_FORWARD | M4_FORWARD;  // F: 0xA5
const uint8_t MOTOR_BACKWARD =
  M1_REVERSE | M2_REVERSE | M3_REVERSE | M4_REVERSE;  // B: 0x5A
const uint8_t MOTOR_MOVE_LEFT =
  M1_REVERSE | M2_FORWARD | M3_FORWARD | M4_REVERSE;  // L
const uint8_t MOTOR_MOVE_RIGHT =
  M1_FORWARD | M2_REVERSE | M3_REVERSE | M4_FORWARD;  // R
const uint8_t MOTOR_CLOCKWISE =
  M1_FORWARD | M2_FORWARD | M3_REVERSE | M4_REVERSE;  // C
const uint8_t MOTOR_COUNTERCLOCKWISE =
  M1_REVERSE | M2_REVERSE | M3_FORWARD | M4_FORWARD;  // K

const unsigned long SERIAL_BAUD = 115200;
const unsigned long COMMAND_TIMEOUT_MS = 500;

unsigned long lastCommandMs = 0;
uint8_t currentMotorData = MOTOR_STOP;


void updateMotor(uint8_t motorData) {
  if (motorData == currentMotorData) {
    return;
  }

  digitalWrite(STCP_PIN, LOW);
  shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, motorData);
  digitalWrite(STCP_PIN, HIGH);
  currentMotorData = motorData;
}


void stopMotors() {
  updateMotor(MOTOR_STOP);
}


bool applyRobotCommand(char command) {
  switch (command) {
    case 'F': updateMotor(MOTOR_FORWARD); return true;
    case 'B': updateMotor(MOTOR_BACKWARD); return true;
    case 'L': updateMotor(MOTOR_MOVE_LEFT); return true;
    case 'R': updateMotor(MOTOR_MOVE_RIGHT); return true;
    // removed: Q, E, Z, X special stop mappings
    case 'C': updateMotor(MOTOR_CLOCKWISE); return true;
    case 'K': updateMotor(MOTOR_COUNTERCLOCKWISE); return true;
    case 'S': stopMotors(); return true;
    default: stopMotors(); return false;
  }
}


void handleCommand(char command) {
  if (!applyRobotCommand(command)) {
    Serial.print("ERR unknown_command ");
    Serial.println(command);
    return;
  }

  lastCommandMs = millis();

  Serial.print("OK ");
  Serial.println(command);
}


void readUsbSerial() {
  while (Serial.available() > 0) {
    char command = static_cast<char>(Serial.read());

    if (command == '\r' || command == '\n' || command == ' ') {
      continue;
    }

    handleCommand(command);
  }
}


void setup() {
  pinMode(DATA_PIN, OUTPUT);
  pinMode(STCP_PIN, OUTPUT);
  pinMode(SHCP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);
  currentMotorData = 255;
  stopMotors();

  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println("READY ESP32_USB_CONTROLLER");
}


void loop() {
  readUsbSerial();

  if (
    lastCommandMs > 0 &&
    millis() - lastCommandMs > COMMAND_TIMEOUT_MS
  ) {
    stopMotors();
    lastCommandMs = 0;
  }
}
