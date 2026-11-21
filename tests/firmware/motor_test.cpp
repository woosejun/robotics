#include <cassert>
#include <iostream>

#include "../../motor_control/esp32_usb_controller/esp32_usb_controller.ino"

FakeSerial Serial;
unsigned long tick = 0;
unsigned long echoUs = 6000;
bool latchWritten = false;
uint8_t latched = 0xff;

void pinMode(int, int) {}
void digitalWrite(int pin, int level) {
  if (pin == EN_PIN && level == DRIVER_ENABLE_LEVEL) {
    assert(latchWritten && latched == 0x00);
  }
}
void shiftOut(int data, int clock, int order, uint8_t mask) {
  assert(data == DATA_PIN && clock == SHCP_PIN && order == MSBFIRST);
  latched = mask;
  latchWritten = true;
}
void delayMicroseconds(unsigned long) {}
unsigned long pulseIn(int, int, unsigned long) { return echoUs; }
unsigned long millis() { return tick; }

void send(const char* commands) {
  for (const char* ch = commands; *ch; ++ch) {
    Serial.bytes.push_back(*ch);
  }
  loop();
}

int main() {
  setup();
  send("F");
  assert(latched == 0x00);
  send("S");

  const struct { char command[2]; uint8_t mask; } expected[] = {
    {"F", 0xA3}, {"B", 0x5C}, {"L", 0x03}, {"R", 0xA0},
    {"C", 0xAC}, {"K", 0x53}, {"S", 0x00},
  };
  for (const auto& item : expected) {
    send(item.command);
    assert(latched == item.mask);
  }
  send("1");
  assert(latched == 0x00);

  send("F");
  assert(latched == 0xA3);
  tick += COMMAND_TIMEOUT_MS;
  send("F");  // queued before expiry was detected
  assert(latched == 0x00);
  send("F");
  assert(latched == 0x00);
  send("SF");
  assert(latched == 0xA3);

  echoUs = 500;  // close obstacle
  tick += PING_INTERVAL_MS;
  send("FF");
  assert(latched == 0x00);
  echoUs = 6000;
  tick += PING_INTERVAL_MS;
  send("SFF");  // queued before the obstacle clearance check
  assert(latched == 0x00);
  send("F");
  assert(latched == 0x00);
  send("SF");
  assert(latched == 0xA3);
  send("S");
  assert(latched == 0x00);

  std::cout << "PASS motor masks, startup, watchdog and obstacle recovery\n";
}
