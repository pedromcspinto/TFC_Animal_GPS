#include "LoRa_APP.h"               // Biblioteca LoRa https://github.com/HelTecAutomation/CubeCell-Arduino
#include <Arduino_JSON.h>           // Biblioteca JSON https://github.com/arduino-libraries/Arduino_JSON
#include <TinyGPSPlus.h>            // Biblioteca GPS https://github.com/mikalhart/TinyGPSPlus

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

#define TIME_TO_SLEEP 10                  // Tempo que o dispositivo dorme em cada ciclo (Em segundos)

#define devEUI "00:00:00:00:00:00:00:02"  // Endereço de identificação do dispositivo
#define boardName "Cubecell 1/2AA Node"   // Nome da placa utilizada
#define application "GPS Tracker"         // Nome da aplicação


typedef enum
{
  LOWPOWER,
  RX,
  SEND_REQUEST,
  SEND_LOCATION
}States_t;
States_t state;

static TimerEvent_t wakeUp;

TinyGPSPlus gps;
bool isLost = false;

String receivedData;
JSONVar receivedJson;
int16_t txNumber;
int16_t Rssi,rxSize;

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
char jsonChArray[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
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
  receivedData = String(rxpacket);
  checkRequestReply();
  if(isLost)
    state = SEND_LOCATION;
  else
    state = LOWPOWER;
  Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n",rxpacket,Rssi,rxSize);
}

// Método transforma um objeto Json num array de caracteres
void jsonToCharArray(JSONVar doc){
  String json = JSON.stringify(doc);
  Serial.println(doc);
  json.toCharArray(jsonChArray, BUFFER_SIZE);
}

// Método que envia a localização
void sendJsonData(){
  JSONVar doc;
  doc["devEUI"] = devEUI;
  doc["application"] = application;
  doc["board"] = boardName;
  doc["data"]["longitude"] = -1;
  doc["data"]["latitude"] = -1;

  // Verifica a posição GPS e adiciona no JSON
  while(Serial1.available()>0){
  if(gps.encode(Serial1.read()))
    if(gps.location.isValid()){
      doc["data"]["longitude"] = gps.location.lat();
      doc["data"]["latitude"] = gps.location.lng();
    }else{
      doc["data"]["longitude"] = -1;
      doc["data"]["latitude"] = -1;
    }     
  }

  // Envia o pacote LoRa da localização
  jsonToCharArray(doc);
  sendPacket(jsonChArray);
}

// Método que envia um request do estado do dispositivo
void sendStatusRequest(){
  JSONVar doc;
  doc["devEUI"] = devEUI;
  doc["application"] = application;
  doc["board"] = boardName;
  doc["data"]["status"] = "request";
  jsonToCharArray(doc);
  sendPacket(jsonChArray);
}

// Método que envia uam sequenia de caracteres num pacote LoRa
void sendPacket(char* data){
  txNumber++;
  Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",data, strlen(data));
  Radio.Send( (uint8_t *)data, strlen(data) );
}
    
// Método que coloca o dispositivo a dormir 
void sleep(){
  TimerSetValue( &wakeUp, TIME_TO_SLEEP * 1000 );
  TimerStart( &wakeUp );
  lowPowerHandler();
}

// Método que é chamado quando o dispositivo acorda
void onWakeUp(){
  state = SEND_REQUEST;
}

// Método que verifica o estado do dispositivo através do pacote recebido
void checkRequestReply(){
  JSONVar doc = JSON.parse(receivedData);
  String destination = doc["destination"];

  // Verifica que o pacote que recebeu é deste dispositivo
  if(destination == devEUI)
    isLost = doc["data"]["status"]; 
}

// Método de inicialização do dispositivo
void setup() {
  Serial.begin(115200);             // Inicialização da comunicação Serial
  Serial1.begin(9600);              // Inicialização da comunicação Serial com o GPS
  txNumber=0;
  Rssi=0;
  
  radioSetup();                     // Inicialização do rádio LoRa
  TimerInit( &wakeUp, onWakeUp );   // Inicializar o timer para o dispositivo dormir
  state = SEND_REQUEST;                                  
}

// Método de funcionamento do dispositivo
void loop() {
  switch (state){

    // Estado que envia o request do estado do dispositivo
    case SEND_REQUEST:
      sendStatusRequest();
      state = RX;
      break;

    // Estado que envia a localização do dispositivo
    case SEND_LOCATION:
      sendJsonData();
      state = LOWPOWER;  
      break;
    
    // Estado que o dispositivo fica em modo de Rx para saber o estado do dispositivo
    case RX:
      Radio.Rx(0);
      state = LOWPOWER;
      break;

    // Estado do dispositivo que o dispositivo entra em modo sleep
    case LOWPOWER:
      Serial.println("Sleeping during " + String(TIME_TO_SLEEP) + " seconds");
      delay(100);
      sleep();
      break;
    default:
      state = LOWPOWER; 
      break; 
  }
  delay(1000);
}