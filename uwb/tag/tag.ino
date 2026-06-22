// DWM3000 tag carried by the person.  It responds to both robot anchors.

#include <SPI.h>
#include "dw3000.h"

const uint8_t PIN_RST = 5;
const uint8_t PIN_IRQ = 4;
const uint8_t PIN_SS = 20;
const uint8_t TAG_ID = 0xAA;

static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8,
    DWT_PHRMODE_STD, DWT_PHRRATE_STD, (129 + 8 - 8),
    DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};
extern dwt_txconfig_t txconfig_options;

static uint8_t tx_resp_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, TAG_ID, 0x00, 0xE1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

#define POLL_RX_TO_RESP_TX_DLY_UUS 1500
#define UUS_TO_DWT_TIME 65536

void setup() {
  Serial.begin(115200);
  delay(1000);
  SPI.begin(21, 22, 23, PIN_SS);
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW); delay(50);
  digitalWrite(PIN_RST, HIGH); delay(500);
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    Serial.println("INIT FAILED");
    while (true) { delay(100); }
  }
  dwt_softreset();
  delay(10);
  dwt_configure(&config);
  dwt_configuretxrf(&txconfig_options);
  dwt_setrxantennadelay(16385);
  dwt_settxantennadelay(16385);
  Serial.println("UWB person tag ready");
}

void loop() {
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  uint32_t status;
  while (!((status = dwt_read32bitreg(SYS_STATUS_ID)) &
           (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {}

  if (status & SYS_STATUS_RXFCG_BIT_MASK) {
    const uint32_t frameLen = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    uint8_t poll[20];
    if (frameLen <= sizeof(poll)) {
      dwt_readrxdata(poll, frameLen, 0);
      if (frameLen >= 10 && poll[9] == 0xE0 && poll[5] == TAG_ID) {
        const uint8_t callerId = poll[7];
        Serial.print("TAG POLL FROM ANCHOR ");
        Serial.println(callerId);
        uint8_t timestampBuffer[5];
        dwt_readrxtimestamp(timestampBuffer);
        uint64_t pollRx = 0;
        for (int i = 4; i >= 0; --i) {
          pollRx = (pollRx << 8) | timestampBuffer[i];
        }

        const uint32_t responseTxTime =
            (pollRx + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(responseTxTime);
        const uint64_t responseTx =
            ((uint64_t)(responseTxTime & 0xFFFFFFFEUL) << 8) + 16385;
        tx_resp_msg[5] = callerId;
        tx_resp_msg[2] = poll[2];
        for (int i = 0; i < 4; ++i) {
          tx_resp_msg[10 + i] = (uint8_t)(pollRx >> (i * 8));
          tx_resp_msg[14 + i] = (uint8_t)(responseTx >> (i * 8));
        }
        dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
        dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
        if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS) {
          while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {}
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        } else {
          Serial.println("TAG RESPONSE TX FAILED");
        }
      }
    }
  } else {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }
}
