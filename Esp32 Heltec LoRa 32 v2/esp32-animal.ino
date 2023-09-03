#include <SPI.h>                  // Biblioteca SPI do Arduino necessária para a comunicação com o modulo LoRa
#include <LoRa.h>                 // Biblioteca LoRa https://github.com/sandeepmistry/arduino-LoRa
#include <Arduino_JSON.h>         // Biblioteca JSON https://github.com/arduino-libraries/Arduino_JSON
#include <TinyGPSPlus.h>          // Biblioteca GPS https://github.com/mikalhart/TinyGPSPlus
#include <SoftwareSerial.h>       // Biblioteca SoftwareSerial do Arduino necessária para a comunicação Serial com o módulo GPS

// Pinos do rśdio LoRa
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

#define BAND 868E6                        // Frequência do rádio LoRa
#define SPREADING_FACTOR 7                // Spreading Factor do rádio LoRa
#define PREAMBLE_LENGTH 8                 // Preamble Length do rádio LoRa
#define BANDWIDTH 125E3                   // Bandwidth do rádio LoRa
#define CODING_RATE 7                     // Coding Rate do rádio LoRa
#define SYNC_WORD 0x12                    // Sync Word do rádio LoRa

#define devEUI "00:00:00:00:00:00:02:01"  // Endereço de identificação do dispositivo
#define boardName "Heltec LoRa 32 v2"     // Nome da placa utilizada
#define application "GPS Tracker"         // Nome da aplicação

#define TIME_TO_SLEEP 10                  // Tempo que o dispositivo dorme em cada ciclo (Em segundos)
#define TIME_TO_SLEEP_LOST 3              // Tempo que o dispositivo dorme em cada ciclo casos esteja perdido (Em segundos)
#define RX_TIMEOUT 3                      // Tempo que o dispositivo vai estar em modo Rx

#define txSensorPin 37                    // Pino onde o Tx do Sensor está conectado
#define rxSensorPin 36                    // Pino onde o Rx do Sensor está conectado

typedef enum
{
  LOWPOWER,
  RECEIVE,
  SEND_REQUEST,
  SEND_LOCATION
}States_t;

RTC_DATA_ATTR States_t state;

bool flager = false;
bool rxFlag;
bool isLost = false;
int rxTime;
int16_t txNumber;
int16_t Rssi,rxSize;

RTC_DATA_ATTR int timeToSleep;
String receivedData;
JSONVar receivedJson;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(txSensorPin, rxSensorPin);


// Método de inicialização do rádio LoRa
void initLoRa(){
  // Definir os pinos do rádio LoRa
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);
  // Parametrização do rádio
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setPreambleLength(PREAMBLE_LENGTH);
  LoRa.setSignalBandwidth(BANDWIDTH);
  LoRa.setCodingRate4(CODING_RATE);
  LoRa.setSyncWord(SYNC_WORD); 
  
  // Verifica a comunicação com o rádio LoRa
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
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
  while(gpsSerial.available()>0){
  if(gps.encode(gpsSerial.read()))
    if(gps.location.isValid()){
        doc["data"]["longitude"] = gps.location.lat();
      doc["data"]["latitude"] = gps.location.lng();
    }else{
      doc["data"]["longitude"] = -1;
      doc["data"]["latitude"] = -1;
    }     
  }

  // Envia o pacote LoRa da localização
  String json = JSON.stringify(doc);
  sendPacket(json);
}

// Método que envia um request do estado do dispositivo
void sendStatusRequest(){
  JSONVar doc;
  doc["devEUI"] = devEUI;
  doc["application"] = application;
  doc["board"] = boardName;
  doc["data"]["status"] = "request";
  String json = JSON.stringify(doc);
  //serializeJson(doc, json);
  sendPacket(json);
}

// Método que envia a String num pacote LoRa
void sendPacket(String data){
  Serial.println(data);
  LoRa.beginPacket();
  LoRa.print(data);
  LoRa.endPacket();
}

// Método que mete o dispositivo a dormir    
void sleep(){
  Serial.println("Sleeping during " + String(timeToSleep) + " seconds");
  esp_deep_sleep_start();
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
  Serial.begin(115200);                                 // Inicialização da comunicação Serial
  gpsSerial.begin(9600);                                // Inicialização da comunicação Serial com o GPS
  initLoRa();                                           // Inicialização do rádio LoRa
  
  state = SEND_REQUEST;                                 
}

// Método de funcionamento do dispositivo
void loop() {
  switch (state){

    // Estado que envia o request do estado do dispositivo
    case SEND_REQUEST:
      sendStatusRequest();
      state = RECEIVE;
      break;

    // Estado que envia a localização do dispositivo
    case SEND_LOCATION:
      delay(1000);
      sendJsonData();
      state = LOWPOWER;
      break;

    // Estado que o dispositivo fica em modo de Rx para saber o estado do dispositivo
    case RECEIVE:
      Serial.println("Receive");
      rxTime = millis();
      rxFlag = false;
      receivedData = "";
      flager = false;

      // Entra em modo Rx
      while(!flager){
        int packetSize = LoRa.parsePacket();
        
        // Verifica se recebeu um pacote
        if(packetSize){
          receivePacket(packetSize);
          rxFlag = true;
        }
        // Verifica se ainda está na janela Rx
        if(millis() - rxTime > RX_TIMEOUT*1000){
          flager = true;
        }
      }
      // Verifica se recebeu um pacote
      if(rxFlag){
        Serial.println("-"+receivedData);
        checkRequestReply();
        // Verifica se o dispositivo está perdido
        if(isLost){
           timeToSleep = TIME_TO_SLEEP_LOST;
           state = SEND_LOCATION;
        }else{
           timeToSleep = TIME_TO_SLEEP;
           state = LOWPOWER;
        }

      }else{
        Serial.println("RX Timeout");
        state = LOWPOWER;
      }
      break;

    // Estado do dispositivo que o dispositivo entra em modo sleep
    case LOWPOWER:
        if(timeToSleep==0)
            timeToSleep=TIME_TO_SLEEP;
        esp_sleep_enable_timer_wakeup(timeToSleep*1000000); // Inicializar o timer para acordar o dispositivo
        sleep();
        break;
    default:
        state = LOWPOWER;
        break;
  }
  delay(200);
}

void receivePacket(int packetSize){
  if(packetSize == 0){
    return;
  } 

  int rssi = LoRa.packetRssi();

  String data = "";
  while(LoRa.available()){
    byte b = LoRa.read();
        if(b >= 32){
            data += (char)b;
        }
  }
  //Serial.println(data);
  receivedData = data;
  //stringToJson(data);
  flager = true;
}