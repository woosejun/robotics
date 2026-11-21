#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <dw3000.h>

extern dwt_txconfig_t txconfig_options;

namespace cart_uwb {

struct RadioPins {
  uint8_t reset;
  uint8_t irq;
  uint8_t sck;
  uint8_t miso;
  uint8_t mosi;
  uint8_t chipSelect;
  uint16_t txAntennaDelay;
  uint16_t rxAntennaDelay;
};

constexpr uint8_t TAG_ID = 0xAA;
constexpr uint8_t POLL = 0xE0;
constexpr uint8_t RESPONSE = 0xE1;
constexpr uint8_t POLL_LENGTH = 12;
constexpr uint8_t RESPONSE_LENGTH = 20;
constexpr uint32_t UUS_TO_DWT_TICKS = 65536UL;
// Leave time for the ESP32 task to notice RX and schedule its delayed response.
constexpr uint32_t TURNAROUND_UUS = 2500;
constexpr uint32_t RX_AFTER_TX_UUS = 300;
constexpr uint32_t RX_TIMEOUT_UUS = 5000;
constexpr uint32_t RX_EVENTS =
    SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO;
constexpr uint32_t TX_EVENT = SYS_STATUS_TXFRS_BIT_MASK;
constexpr double METRES_PER_TICK = 299702547.0 / (499.2e6 * 128.0);

inline void initFailed(const char* reason) {
  for (;;) {
    Serial.printf("UWB INIT FAILED: %s\n", reason);
    delay(1000);
  }
}

inline void initRadio(const RadioPins& pins) {
  Serial.begin(115200);
  if (!digitalPinCanOutput(pins.reset) || !digitalPinIsValid(pins.irq)
      || !digitalPinCanOutput(pins.sck) || !digitalPinIsValid(pins.miso)
      || !digitalPinCanOutput(pins.mosi) || !digitalPinCanOutput(pins.chipSelect)) {
    initFailed("the configured GPIOs are invalid for this ESP32 variant");
  }
  SPI.begin(pins.sck, pins.miso, pins.mosi, pins.chipSelect);
  spiBegin(pins.irq, pins.reset);
  spiSelect(pins.chipSelect);  // The port library handles hardware reset.
  const unsigned long started = millis();
  while (!dwt_checkidlerc()) {
    if (millis() - started >= 500) {
      initFailed("DW3000 did not enter IDLE_RC; check SPI, reset and power");
    }
    delay(5);
  }

  dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8,
    DWT_PHRMODE_STD, DWT_PHRRATE_STD, 129, DWT_STS_MODE_OFF,
    DWT_STS_LEN_64, DWT_PDOA_M0,
  };
  if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS || dwt_configure(&config) != DWT_SUCCESS) {
    initFailed("DW3000 initialization or PHY configuration");
  }
  dwt_configuretxrf(&txconfig_options);
  dwt_settxantennadelay(pins.txAntennaDelay);
  dwt_setrxantennadelay(pins.rxAntennaDelay);
}

inline uint32_t waitForEvent(uint32_t mask, unsigned long timeoutMs) {
  const unsigned long started = millis();
  do {
    const uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
    if (status & mask) {
      return status;
    }
    delayMicroseconds(60);
  } while (millis() - started < timeoutMs);
  return 0;
}

inline void resetTransceiver() {
  dwt_forcetrxoff();
  dwt_write32bitreg(SYS_STATUS_ID, RX_EVENTS | TX_EVENT);
}

inline void makeHeader(uint8_t* frame, uint8_t seq, uint8_t to, uint8_t from, uint8_t type) {
  frame[0] = 0x41;
  frame[1] = 0x88;
  frame[2] = seq;
  frame[3] = 0xCA;
  frame[4] = 0xDE;
  frame[5] = to;
  frame[6] = 0;
  frame[7] = from;
  frame[8] = 0;
  frame[9] = type;
}

inline bool matchesHeader(const uint8_t* frame, uint8_t to, uint8_t from, uint8_t type) {
  uint8_t expected[10];
  makeHeader(expected, frame[2], to, from, type);
  return memcmp(frame, expected, sizeof(expected)) == 0;
}

inline uint32_t readTimestamp32(const uint8_t* bytes) {
  uint32_t value = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    value |= static_cast<uint32_t>(bytes[i]) << (i * 8);
  }
  return value;
}

inline void writeTimestamp32(uint8_t* bytes, uint64_t value) {
  for (uint8_t i = 0; i < 4; ++i) {
    bytes[i] = static_cast<uint8_t>(value >> (i * 8));
  }
}

inline void sendFrame(uint8_t* frame, uint8_t length) {
  dwt_writetxdata(length, frame, 0);
  dwt_writetxfctrl(length, 0, 1);
}

inline void initAnchor(const RadioPins& pins, uint8_t anchorId) {
  initRadio(pins);
  dwt_setrxaftertxdelay(RX_AFTER_TX_UUS);
  dwt_setrxtimeout(RX_TIMEOUT_UUS);
  Serial.printf("ANCHOR%u READY\n", static_cast<unsigned>(anchorId));
}

inline bool measureRange(uint8_t anchorId, uint8_t seq, double& distance) {
  resetTransceiver();
  uint8_t poll[POLL_LENGTH] = {};
  makeHeader(poll, seq, TAG_ID, anchorId, POLL);
  sendFrame(poll, sizeof(poll));
  if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
    return false;
  }
  if (!(waitForEvent(RX_EVENTS, 20) & SYS_STATUS_RXFCG_BIT_MASK)) {
    resetTransceiver();
    return false;
  }
  if ((dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK) != RESPONSE_LENGTH) {
    return false;
  }
  uint8_t response[RESPONSE_LENGTH];
  dwt_readrxdata(response, sizeof(response), 0);
  if (response[2] != seq || !matchesHeader(response, anchorId, TAG_ID, RESPONSE)) {
    return false;
  }

  const uint32_t pollTx = dwt_readtxtimestamplo32();
  const uint32_t respRx = dwt_readrxtimestamplo32();
  const uint32_t pollRx = readTimestamp32(response + 10);
  const uint32_t respTx = readTimestamp32(response + 14);
  // Unsigned subtraction handles the DW3000 32-bit timestamp wrap.
  const uint32_t roundTrip = respRx - pollTx;
  const uint32_t replyDelay = respTx - pollRx;
  const double clockOffset = dwt_readclockoffset() / static_cast<double>(1UL << 26);
  distance = (roundTrip - replyDelay * (1.0 - clockOffset)) * 0.5 * METRES_PER_TICK;
  return distance >= 0.05 && distance <= 100.0;
}

inline void runAnchor(uint8_t anchorId, unsigned long periodMs) {
  static uint8_t sequence = 0;
  const unsigned long start = millis();
  double distance;
  if (measureRange(anchorId, sequence++, distance)) {
    Serial.printf("ANCHOR%u,%.3f\n", static_cast<unsigned>(anchorId), distance);
  } else {
    Serial.printf("ANCHOR%u NO VALID RESPONSE\n", static_cast<unsigned>(anchorId));
  }

  // Different periods and local jitter prevent a collision from repeating in lockstep.
  const unsigned long interval = periodMs + random(0, 19);
  const unsigned long elapsed = millis() - start;
  delay(elapsed < interval ? interval - elapsed : 1);
}

inline void initTag(const RadioPins& pins) {
  initRadio(pins);
  dwt_setrxtimeout(0);
  Serial.println("UWB TAG READY");
  if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
    initFailed("DW3000 could not start tag RX");
  }
}

inline void runTag(uint16_t txAntennaDelay) {
  const uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
  if (!(status & RX_EVENTS)) {
    // Keep RX armed across Arduino loop iterations when no frame has arrived.
    delay(1);
    return;
  }

  if (status & SYS_STATUS_RXFCG_BIT_MASK) {
    const uint32_t length = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    uint8_t poll[POLL_LENGTH];
    if (length == sizeof(poll)) {
      dwt_readrxdata(poll, sizeof(poll), 0);
      const uint8_t anchorId = poll[7];
      if ((anchorId == 1 || anchorId == 2) && matchesHeader(poll, TAG_ID, anchorId, POLL)) {
        uint8_t bytes[5];
        dwt_readrxtimestamp(bytes);
        const uint64_t pollRx = (static_cast<uint64_t>(bytes[4]) << 32) | readTimestamp32(bytes);
        const uint32_t txTime = (pollRx + TURNAROUND_UUS * UUS_TO_DWT_TICKS) >> 8;

        uint8_t response[RESPONSE_LENGTH] = {};
        makeHeader(response, poll[2], anchorId, TAG_ID, RESPONSE);
        writeTimestamp32(response + 10, pollRx);
        const uint64_t respTx = (static_cast<uint64_t>(txTime & ~1UL) << 8) + txAntennaDelay;
        writeTimestamp32(response + 14, respTx);
        resetTransceiver();
        dwt_setdelayedtrxtime(txTime);
        sendFrame(response, sizeof(response));
        if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS
            || !(waitForEvent(TX_EVENT, 10) & TX_EVENT)) {
          Serial.println("TAG RESPONSE TX FAILED");
        }
      }
    }
  }
  resetTransceiver();
  if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
    initFailed("DW3000 could not resume tag RX");
  }
}

}  // namespace cart_uwb
