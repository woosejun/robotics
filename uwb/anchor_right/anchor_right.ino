// 기존 앵커 2번(오른쪽) DWM3000 코드. 오른쪽 보드는 CS 핀이 20번이다.
#include <SPI.h>
#include "dw3000.h"
const uint8_t PIN_RST=5, PIN_IRQ=4, PIN_SS=20;
static dwt_config_t config={5,DWT_PLEN_128,DWT_PAC8,9,9,1,DWT_BR_6M8,DWT_PHRMODE_STD,DWT_PHRRATE_STD,(129+8-8),DWT_STS_MODE_OFF,DWT_STS_LEN_64,DWT_PDOA_M0};
extern dwt_txconfig_t txconfig_options;
static uint8_t tx_poll_msg[]={0x41,0x88,0,0xCA,0xDE,0xAA,0x00,0x02,0x00,0xE0,0,0};
static uint8_t frame_seq_nb=0;
static const uint16_t POLL_PERIOD_MS = 250;
#define SPEED_OF_LIGHT 299702547
#define DWT_TIME_UNITS (1.0/499.2e6/128.0)
void setup(){
  Serial.begin(115200); delay(1000); SPI.begin(21,22,23,PIN_SS);
  pinMode(PIN_RST,OUTPUT); digitalWrite(PIN_RST,LOW); delay(50); digitalWrite(PIN_RST,HIGH); delay(500);
  spiBegin(PIN_IRQ,PIN_RST); spiSelect(PIN_SS);
  if(dwt_initialise(DWT_DW_INIT)==DWT_ERROR){Serial.println("INIT FAILED");while(1){delay(100);}}
  dwt_configure(&config); dwt_configuretxrf(&txconfig_options); dwt_setrxantennadelay(16385); dwt_settxantennadelay(16385);
  Serial.println("UWB right anchor ready");
  // Keep this anchor half a period away from anchor 1.  Both anchors using
  // nearly the same period causes their polls to collide at the single tag.
  delay(POLL_PERIOD_MS / 2);
}
void loop(){
  tx_poll_msg[2]=frame_seq_nb; dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_TXFRS_BIT_MASK);
  dwt_writetxdata(sizeof(tx_poll_msg),tx_poll_msg,0); dwt_writetxfctrl(sizeof(tx_poll_msg),0,1);
  if(dwt_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED)!=DWT_SUCCESS){
    Serial.println("ANCHOR2 TX START FAILED");
    delay(POLL_PERIOD_MS);
    return;
  }
  uint32_t status_reg=0,timeout_counter=0; bool timeout_occurred=false;
  while(!((status_reg=dwt_read32bitreg(SYS_STATUS_ID))&(SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_ERR))){
    delay(1);if(++timeout_counter>20){timeout_occurred=true;break;}
  }
  if(!timeout_occurred&&(status_reg&SYS_STATUS_RXFCG_BIT_MASK)){
    uint32_t frame_len=dwt_read32bitreg(RX_FINFO_ID)&RXFLEN_MASK;dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_RXFCG_BIT_MASK);uint8_t rx_buffer[20];
    if(frame_len>=18&&frame_len<=sizeof(rx_buffer)){
      dwt_readrxdata(rx_buffer,frame_len,0);
      if(rx_buffer[9]==0xE1&&rx_buffer[5]==0x02&&rx_buffer[7]==0xAA){
        uint32_t poll_tx_ts=dwt_readtxtimestamplo32(),resp_rx_ts=dwt_readrxtimestamplo32(),poll_rx_ts=0,resp_tx_ts=0;
        for(int i=0;i<4;i++){poll_rx_ts+=(uint32_t)rx_buffer[10+i]<<(i*8);resp_tx_ts+=(uint32_t)rx_buffer[14+i]<<(i*8);}
        float clockOffsetRatio=((float)dwt_readclockoffset())/(uint32_t)(1<<26);int32_t rtd_init=resp_rx_ts-poll_tx_ts,rtd_resp=resp_tx_ts-poll_rx_ts;
        float distance=(((rtd_init-rtd_resp*(1-clockOffsetRatio))/2.0)*DWT_TIME_UNITS)*SPEED_OF_LIGHT;
        Serial.print("ANCHOR2,");Serial.println(distance);
      }
    }
  }else if(status_reg&SYS_STATUS_ALL_RX_ERR){
    Serial.print("ANCHOR2 RX ERROR 0x"); Serial.println(status_reg, HEX);
    dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_ALL_RX_ERR);
  }
  if(timeout_occurred){
    Serial.println("ANCHOR2 NO RESPONSE");
  }
  delay(POLL_PERIOD_MS);frame_seq_nb++;
}
