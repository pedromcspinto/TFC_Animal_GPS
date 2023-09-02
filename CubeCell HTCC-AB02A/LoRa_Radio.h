//#include "utils.h"

#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

#define RF_FREQUENCY                                868000000 // Frequência do rádio LoRa (868MHz)
#define TX_OUTPUT_POWER                             5         // Potência de transmissão do rádio Lora (dbm)
#define LORA_BANDWIDTH                              0         // Bandwidth do rádio LoRa ([0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved])
#define LORA_SPREADING_FACTOR                       7         // Spreading Factor do rádio LoRa
#define LORA_CODINGRATE                             1         // Coding Rate do rádio LoRa([1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8])
#define LORA_PREAMBLE_LENGTH                        8         // Preamble Length do rádio LoRa
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false     
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            2000      // Tempo que o rádio fica na janela de Rx (ms)
#define BUFFER_SIZE                                 250       // Número de bytes máximo do pacote

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
int16_t txNumber;
bool sleepMode = false;
int16_t Rssi,rxSize;

void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

// Método de inicialização do rádio LoRa
void radioSetup(){
  Radio.Sleep( );

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                  LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                  LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                  0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

}

// Método que é chamado quando o rádio LoRa acabar de transmitir
void OnTxDone( void ){
  Serial.println("...TX Timeout");
}

// Método que é chamado quando o rádio LoRa acabar de transmitir
void OnTxTimeout( void ){
  Radio.Sleep();
  //Serial.println("...TX Timeout");
}

// Método que é chamado quando o rádio LoRa receber um pacote
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ){
  Rssi=rssi;
  rxSize=size;
  memcpy(rxpacket, payload, size);
  rxpacket[size]='\0';
  Radio.Sleep();
  stringToJson(String(rxpacket));
  Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n",rxpacket,Rssi,rxSize);
}