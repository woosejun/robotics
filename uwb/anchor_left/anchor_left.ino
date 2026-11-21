// Competition LEFT anchor (ANCHOR1). Verify this profile against the actual PCB.
#include <cart_uwb.h>

constexpr uint8_t ANCHOR_ID = 1;
constexpr cart_uwb::RadioPins BOARD = {
  5, 4, 21, 22, 23, 2, 16385, 16385  // RST, IRQ, SCK, MISO, MOSI, CS, TX delay, RX delay
};

void setup() {
  cart_uwb::initAnchor(BOARD, ANCHOR_ID);
}

void loop() {
  cart_uwb::runAnchor(ANCHOR_ID, 230);
}
