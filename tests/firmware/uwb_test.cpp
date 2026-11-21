#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "../../uwb/libraries/CartUwb/src/cart_uwb.h"

FakeSerial Serial;
FakeSpi SPI;
dwt_txconfig_t txconfig_options;
uint32_t status = 0;
uint32_t delayedAt = 0;
unsigned long clockUs = 0;
uint64_t tagRx = 0x16fffff010ULL;
uint32_t anchorTx = 0xffffff00U;
uint32_t anchorRx = 0;
int rxActivations = 0;
std::vector<uint8_t> received, transmitted, reply;

unsigned long millis() { return clockUs / 1000; }
void delay(unsigned long ms) { clockUs += ms * 1000; }
void delayMicroseconds(unsigned long us) { clockUs += us; }
long random(long low, long) { return low; }
bool digitalPinCanOutput(uint8_t) { return true; }
bool digitalPinIsValid(uint8_t) { return true; }
void spiBegin(uint8_t, uint8_t) {}
void spiSelect(uint8_t) {}
int dwt_checkidlerc() { return 1; }
int dwt_initialise(int) { return DWT_SUCCESS; }
int dwt_configure(dwt_config_t*) { return DWT_SUCCESS; }
void dwt_configuretxrf(dwt_txconfig_t*) {}
void dwt_settxantennadelay(uint16_t) {}
void dwt_setrxantennadelay(uint16_t) {}
void dwt_setrxaftertxdelay(uint32_t) {}
void dwt_setrxtimeout(uint32_t) {}
void dwt_forcetrxoff() {}
uint32_t dwt_read32bitreg(int reg) {
  return reg == SYS_STATUS_ID ? status : static_cast<uint32_t>(received.size());
}
void dwt_write32bitreg(int, uint32_t bits) { status &= ~bits; }
void dwt_writetxdata(uint16_t length, const uint8_t* data, uint16_t) {
  transmitted.assign(data, data + length);
}
void dwt_writetxfctrl(uint16_t, uint16_t, int) {}
int dwt_starttx(int mode) {
  status = SYS_STATUS_TXFRS_BIT_MASK;
  if ((mode & DWT_RESPONSE_EXPECTED) && !reply.empty()) {
    received = reply;
    status |= SYS_STATUS_RXFCG_BIT_MASK;
  }
  return DWT_SUCCESS;
}
void dwt_readrxdata(uint8_t* data, uint16_t length, uint16_t offset) {
  assert(offset + length <= received.size());
  std::memcpy(data, received.data() + offset, length);
}
uint32_t dwt_readtxtimestamplo32() { return anchorTx; }
uint32_t dwt_readrxtimestamplo32() { return anchorRx; }
int dwt_readclockoffset() { return 0; }
void dwt_readrxtimestamp(uint8_t* data) {
  for (int i = 0; i < 5; ++i) {
    data[i] = static_cast<uint8_t>(tagRx >> (8 * i));
  }
}
void dwt_setdelayedtrxtime(uint32_t at) { delayedAt = at; }
int dwt_rxenable(int) { ++rxActivations; return DWT_SUCCESS; }

int main() {
  using namespace cart_uwb;
  constexpr uint16_t antennaDelay = 16385;
  constexpr uint32_t flightTicks = 256;
  for (uint8_t anchor : {1, 2}) {
    const uint8_t seq = 41 + anchor;
    received.assign(POLL_LENGTH, 0);
    makeHeader(received.data(), seq, TAG_ID, anchor, POLL);
    status = SYS_STATUS_RXFCG_BIT_MASK;
    const int before = rxActivations;
    runTag(antennaDelay);
    assert(rxActivations == before + 1);
    assert(delayedAt == static_cast<uint32_t>((tagRx + TURNAROUND_UUS * UUS_TO_DWT_TICKS) >> 8));
    assert(transmitted.size() == RESPONSE_LENGTH);
    assert(transmitted[2] == seq && matchesHeader(transmitted.data(), anchor, TAG_ID, RESPONSE));

    reply = transmitted;
    const uint32_t pollRx = readTimestamp32(reply.data() + 10);
    const uint32_t replyDelay = readTimestamp32(reply.data() + 14) - pollRx;
    assert(pollRx == static_cast<uint32_t>(tagRx));
    anchorRx = anchorTx + replyDelay + 2 * flightTicks;
    assert(anchorRx < anchorTx);  // simulated 32-bit timestamp wrap

    double range;
    assert(measureRange(anchor, seq, range));
    assert(std::abs(range - flightTicks * METRES_PER_TICK) < 1e-9);
    assert(transmitted[2] == seq && matchesHeader(transmitted.data(), TAG_ID, anchor, POLL));
    assert(!measureRange(anchor, seq + 1, range));
    reply[5] = anchor == 1 ? 2 : 1;
    assert(!measureRange(anchor, seq, range));

    reply.clear();
    const unsigned long started = millis();
    assert(!measureRange(anchor, seq, range));
    assert(millis() - started == 20 && status == 0);
  }
  std::cout << "PASS both UWB anchor polls, tag replies, range, sequence and timeout\n";
}
