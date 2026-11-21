// Competition RIGHT anchor (ANCHOR2). Check GPIO20 against the board's USB wiring.
#include <cart_uwb.h>

constexpr uint8_t ANCHOR_ID = 2;
constexpr cart_uwb::RadioPins BOARD = {
  5, 4, 21, 22, 23, 20, 16385, 16385  // RST, IRQ, SCK, MISO, MOSI, CS, TX delay, RX delay
};

void setup() {
  cart_uwb::initAnchor(BOARD, ANCHOR_ID);
  delay(115);
}

void loop() {
  cart_uwb::runAnchor(ANCHOR_ID, 270);
}
