#pragma once
#include <cstdint>
#include <cstring>

struct dwt_config_t { int values[13]; };
struct dwt_txconfig_t {};

constexpr int DWT_SUCCESS = 0;
constexpr int DWT_DW_INIT = 0;
constexpr int DWT_PLEN_128 = 128;
constexpr int DWT_PAC8 = 8;
constexpr int DWT_BR_6M8 = 1;
constexpr int DWT_PHRMODE_STD = 0;
constexpr int DWT_PHRRATE_STD = 0;
constexpr int DWT_STS_MODE_OFF = 0;
constexpr int DWT_STS_LEN_64 = 64;
constexpr int DWT_PDOA_M0 = 0;
constexpr int DWT_START_TX_IMMEDIATE = 0;
constexpr int DWT_START_TX_DELAYED = 1;
constexpr int DWT_RESPONSE_EXPECTED = 2;
constexpr int DWT_START_RX_IMMEDIATE = 0;
constexpr int SYS_STATUS_ID = 0;
constexpr int RX_FINFO_ID = 1;
constexpr int RXFLEN_MASK = 0x3ff;
constexpr uint32_t SYS_STATUS_RXFCG_BIT_MASK = 1;
constexpr uint32_t SYS_STATUS_ALL_RX_ERR = 2;
constexpr uint32_t SYS_STATUS_ALL_RX_TO = 4;
constexpr uint32_t SYS_STATUS_TXFRS_BIT_MASK = 8;

void spiBegin(uint8_t, uint8_t);
void spiSelect(uint8_t);
int dwt_checkidlerc();
int dwt_initialise(int);
int dwt_configure(dwt_config_t*);
void dwt_configuretxrf(dwt_txconfig_t*);
void dwt_settxantennadelay(uint16_t);
void dwt_setrxantennadelay(uint16_t);
void dwt_setrxaftertxdelay(uint32_t);
void dwt_setrxtimeout(uint32_t);
void dwt_forcetrxoff();
uint32_t dwt_read32bitreg(int);
void dwt_write32bitreg(int, uint32_t);
void dwt_writetxdata(uint16_t, const uint8_t*, uint16_t);
void dwt_writetxfctrl(uint16_t, uint16_t, int);
int dwt_starttx(int);
void dwt_readrxdata(uint8_t*, uint16_t, uint16_t);
uint32_t dwt_readtxtimestamplo32();
uint32_t dwt_readrxtimestamplo32();
int dwt_readclockoffset();
void dwt_readrxtimestamp(uint8_t*);
void dwt_setdelayedtrxtime(uint32_t);
int dwt_rxenable(int);
