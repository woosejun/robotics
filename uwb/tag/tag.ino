// Person-carried tag. Check GPIO20 against the board's USB wiring.
#include <cart_uwb.h>

constexpr cart_uwb::RadioPins BOARD = {
  5, 4, 21, 22, 23, 20, 16385, 16385  // RST, IRQ, SCK, MISO, MOSI, CS, TX delay, RX delay
};

void setup() {
  cart_uwb::initTag(BOARD);
}

void loop() {
  cart_uwb::runTag(BOARD.txAntennaDelay);
}
