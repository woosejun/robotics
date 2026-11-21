#pragma once
#include <cstdint>

struct FakeSpi {
  void begin(uint8_t, uint8_t, uint8_t, uint8_t) {}
};

extern FakeSpi SPI;
