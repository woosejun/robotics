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
const uint8_t M3_FORWARD = 0x10;
const uint8_t M3_REVERSE = 0x20;
const uint8_t M4_FORWARD = 0x80;
const uint8_t M4_REVERSE = 0x40;

// --------------------------------------------------
// 실제 측정한 AceBot 74HC595 매핑
//
// 0x01 = 오른쪽 뒤 전진
// 0x02 = 오른쪽 앞 전진
// 0x04 = 오른쪽 앞 후진
// 0x08 = 오른쪽 뒤 후진
// 0x10 = 왼쪽 뒤 후진
// 0x20 = 왼쪽 뒤 전진
// 0x40 = 왼쪽 앞 후진
// 0x80 = 왼쪽 앞 전진
// --------------------------------------------------
const uint8_t MOTOR_STOP = 0;
// 직진
const uint8_t MOTOR_FORWARD =
  0x01 | 0x02 | 0x20 | 0x80;   // 0xA3
// 후진
const uint8_t MOTOR_BACKWARD =
  0x04 | 0x08 | 0x10 | 0x40;   // 0x5C
// 좌코너링 (오른쪽 바퀴만 전진)
const uint8_t MOTOR_MOVE_LEFT =
  0x01 | 0x02;                 // 0x03
// 우코너링 (왼쪽 바퀴만 전진)
const uint8_t MOTOR_MOVE_RIGHT =
  0x20 | 0x80;                 // 0xA0
// 제자리 우회전
const uint8_t MOTOR_CLOCKWISE =
  0x04 | 0x08 | 0x20 | 0x80;   // 0xAC
// 제자리 좌회전
const uint8_t MOTOR_COUNTERCLOCKWISE =
  0x01 | 0x02 | 0x10 | 0x40;   // 0x53

// 초음파 + 부저
const int trigPin = 13;
const int echoPin = 14;
const int buzzerPin = 12;
const float STOP_DISTANCE_CM = 20.0;

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


float readDistanceCM(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) {
    return -1;
  }
  return duration * 0.0343 / 2.0;
}


char normalizeRobotCommand(char command) {
  if (command >= 'a' && command <= 'z') {
    command = command - 'a' + 'A';
  }

  switch (command) {
    case 'W': return 'F';
    case 'A': return 'L';
    case 'D': return 'R';
    case 'X': return 'B';
    default: return command;
  }
}


bool applyRobotCommand(char command) {
  command = normalizeRobotCommand(command);

  switch (command) {
    // 비트 매핑 테스트: 1~8은 각 비트 직접 제어
    case '1': updateMotor(0x01); return true;
    case '2': updateMotor(0x02); return true;
    case '3': updateMotor(0x04); return true;
    case '4': updateMotor(0x08); return true;
    case '5': updateMotor(0x10); return true;
    case '6': updateMotor(0x20); return true;
    case '7': updateMotor(0x40); return true;
    case '8': updateMotor(0x80); return true;
    case 'F': updateMotor(MOTOR_FORWARD); return true;
    case 'B': updateMotor(MOTOR_BACKWARD); return true;
    case 'L': updateMotor(MOTOR_MOVE_LEFT); return true;
    case 'R': updateMotor(MOTOR_MOVE_RIGHT); return true;
    case 'C': updateMotor(MOTOR_CLOCKWISE); return true;
    case 'K': updateMotor(MOTOR_COUNTERCLOCKWISE); return true;
    case 'S': stopMotors(); return true;
    default: stopMotors(); return false;
  }
}


void handleCommand(char command) {
  char originalCommand = command;
  char normalizedCommand = normalizeRobotCommand(command);

  if (!applyRobotCommand(command)) {
    Serial.print("ERR unknown_command ");
    Serial.println(originalCommand);
    return;
  }

  lastCommandMs = millis();

  Serial.print("OK ");
  Serial.println(normalizedCommand);
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
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  digitalWrite(EN_PIN, LOW);
  currentMotorData = 255;
  stopMotors();

  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println("READY ESP32_USB_CONTROLLER");
}


void loop() {
  float distance = readDistanceCM();
  if (
    distance > 0 &&
    distance < STOP_DISTANCE_CM
  ) {
    stopMotors();
    digitalWrite(buzzerPin, HIGH);
    return;
  }

  digitalWrite(buzzerPin, LOW);

  readUsbSerial();

  if (
    lastCommandMs > 0 &&
    millis() - lastCommandMs > COMMAND_TIMEOUT_MS
  ) {
    stopMotors();
    lastCommandMs = 0;
  }
}
