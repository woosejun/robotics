// Feed the production sketch commands over a simulated USB endpoint.
#include <chrono>
#include <iostream>
#include <string>

#include "../../motor_control/esp32_usb_controller/esp32_usb_controller.ino"

FakeSerial Serial;
uint8_t outputMask = 0xff;
const auto boot = std::chrono::steady_clock::now();

void pinMode(int, int) {}
void digitalWrite(int, int) {}
void shiftOut(int, int, int, uint8_t mask) { outputMask = mask; }
void delayMicroseconds(unsigned long) {}
unsigned long pulseIn(int, int, unsigned long) { return 6000; }
unsigned long millis() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - boot).count();
}

int main() {
  setup();
  std::string line;
  while (std::getline(std::cin, line)) {
    Serial.output.clear();
    for (char command : line) {
      Serial.bytes.push_back(command);
    }
    loop();
    std::cout << Serial.output << "MASK " << static_cast<unsigned>(outputMask) << std::endl;
  }
}
