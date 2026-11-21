#include <Arduino.h>

// Hardware profile: AceBot shield with a 74HC595, competition wiring.
// Change these GPIOs, the enable polarity and the wheel bits for another shield.
constexpr uint8_t DATA_PIN = 5;
constexpr uint8_t STCP_PIN = 17;
constexpr uint8_t SHCP_PIN = 18;
constexpr uint8_t EN_PIN = 16;
constexpr uint8_t DRIVER_ENABLE_LEVEL = LOW;
constexpr uint8_t DRIVER_DISABLE_LEVEL = HIGH;

// Measured bit-to-wheel map. Left and right are viewed from behind the cart.
// Forward and reverse describe wheel motion with the original motor wiring.
constexpr uint8_t REAR_RIGHT_FORWARD = 0x01;
constexpr uint8_t FRONT_RIGHT_FORWARD = 0x02;
constexpr uint8_t FRONT_RIGHT_REVERSE = 0x04;
constexpr uint8_t REAR_RIGHT_REVERSE = 0x08;
constexpr uint8_t REAR_LEFT_REVERSE = 0x10;
constexpr uint8_t REAR_LEFT_FORWARD = 0x20;
constexpr uint8_t FRONT_LEFT_REVERSE = 0x40;
constexpr uint8_t FRONT_LEFT_FORWARD = 0x80;

constexpr uint8_t LEFT_FORWARD = FRONT_LEFT_FORWARD | REAR_LEFT_FORWARD;
constexpr uint8_t LEFT_REVERSE = FRONT_LEFT_REVERSE | REAR_LEFT_REVERSE;
constexpr uint8_t RIGHT_FORWARD = FRONT_RIGHT_FORWARD | REAR_RIGHT_FORWARD;
constexpr uint8_t RIGHT_REVERSE = FRONT_RIGHT_REVERSE | REAR_RIGHT_REVERSE;

constexpr uint8_t MOTOR_STOP = 0x00;
constexpr uint8_t MOTOR_FORWARD = LEFT_FORWARD | RIGHT_FORWARD;                  // 0xA3
constexpr uint8_t MOTOR_BACKWARD = LEFT_REVERSE | RIGHT_REVERSE;                  // 0x5C
constexpr uint8_t MOTOR_LEFT_ARC = RIGHT_FORWARD;                                 // 0x03
constexpr uint8_t MOTOR_RIGHT_ARC = LEFT_FORWARD;                                 // 0xA0
constexpr uint8_t MOTOR_CLOCKWISE = LEFT_FORWARD | RIGHT_REVERSE;                 // 0xAC
constexpr uint8_t MOTOR_COUNTERCLOCKWISE = LEFT_REVERSE | RIGHT_FORWARD;          // 0x53

// Hardware profile: front ultrasonic sensor and buzzer.
constexpr uint8_t TRIGGER_PIN = 13;
constexpr uint8_t ECHO_PIN = 14;
constexpr uint8_t BUZZER_PIN = 12;
constexpr float STOP_DISTANCE_CM = 20.0f;
// An open field and a disconnected sensor can both return no echo.
constexpr bool STOP_ON_MISSING_ECHO = false;
// Set true only for wheel-on-stand bit mapping. Send raw '1' through '8'.
constexpr bool ENABLE_BIT_MAPPING_TEST = false;

constexpr unsigned long SERIAL_BAUD = 115200;
constexpr unsigned long COMMAND_TIMEOUT_MS = 500;
constexpr unsigned long ECHO_TIMEOUT_US = 30000;
constexpr unsigned long PING_INTERVAL_MS = 60;

unsigned long lastCommandMs = 0;
unsigned long lastPingMs = 0;
uint8_t currentMotorData = 0xff;
bool commandActive = false;
bool obstacleBlocked = false;
bool requiresStopAck = true;
bool sensorChecked = false;

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
  commandActive = false;
}

float readDistanceCM() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  const unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  return duration == 0 ? -1.0f : duration * 0.0343f / 2.0f;
}

char normalizeRobotCommand(char command) {
  if (command >= 'a' && command <= 'z') {
    command -= 'a' - 'A';
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
  if (ENABLE_BIT_MAPPING_TEST && command >= '1' && command <= '8') {
    updateMotor(static_cast<uint8_t>(1U << (command - '1')));
    return true;
  }
  switch (command) {
    case 'F': updateMotor(MOTOR_FORWARD); return true;
    case 'B': updateMotor(MOTOR_BACKWARD); return true;
    case 'L': updateMotor(MOTOR_LEFT_ARC); return true;
    case 'R': updateMotor(MOTOR_RIGHT_ARC); return true;
    case 'C': updateMotor(MOTOR_CLOCKWISE); return true;
    case 'K': updateMotor(MOTOR_COUNTERCLOCKWISE); return true;
    default: return false;
  }
}

void acknowledge(char command) {
  Serial.print("OK ");
  Serial.println(command);
}

void handleCommand(char command) {
  command = normalizeRobotCommand(command);
  if (command == 'S') {
    stopMotors();
    if (!obstacleBlocked) {
      requiresStopAck = false;
    }
    acknowledge(command);
  } else if (!obstacleBlocked && !requiresStopAck) {
    if (applyRobotCommand(command)) {
      lastCommandMs = millis();
      commandActive = true;
      acknowledge(command);
    } else {
      stopMotors();
      Serial.println("ERR unknown_command");
    }
  }
}

void readUsbSerial(bool discard) {
  // Process only the bytes already queued when this check began.
  const int queued = Serial.available();
  for (int i = 0; i < queued; ++i) {
    const int value = Serial.read();
    if (!discard && value > ' ') {
      handleCommand(static_cast<char>(value));
    }
  }
}

void setup() {
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, DRIVER_DISABLE_LEVEL);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(STCP_PIN, OUTPUT);
  pinMode(SHCP_PIN, OUTPUT);
  stopMotors();  // latch the stop mask before enabling the driver
  digitalWrite(EN_PIN, DRIVER_ENABLE_LEVEL);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.begin(SERIAL_BAUD);
  Serial.println("READY ESP32_USB_CONTROLLER");
}

void loop() {
  const unsigned long now = millis();
  bool justCleared = false;
  if (!sensorChecked || now - lastPingMs >= PING_INTERVAL_MS) {
    lastPingMs = now;
    sensorChecked = true;
    const float distance = readDistanceCM();
    const bool blocked = distance < 0 ? STOP_ON_MISSING_ECHO : distance < STOP_DISTANCE_CM;
    justCleared = obstacleBlocked && !blocked;
    obstacleBlocked = blocked;
    if (obstacleBlocked) {
      stopMotors();
      requiresStopAck = true;
    }
    digitalWrite(BUZZER_PIN, obstacleBlocked ? HIGH : LOW);
  }

  if (commandActive && millis() - lastCommandMs >= COMMAND_TIMEOUT_MS) {
    stopMotors();
    requiresStopAck = true;
    readUsbSerial(true);
    return;
  }
  // Drop bytes queued before the clearance measurement. Require a new S
  // before accepting motion again, then accept only commands received after it.
  readUsbSerial(justCleared);
}
