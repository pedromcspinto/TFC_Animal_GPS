/*
  Para programar ligar a um módulo FTDI
  FDTI      Lora Node
  
  DTR       DTR
  TX        RX
  RX        TX
  VCC       3V3
  GND       GND

*/

#include <SPI.h>                  // Biblioteca SPI do Arduino necessária para a comunicação com o modulo LoRa
#include <LoRa.h>                 // Biblioteca LoRa https://github.com/sandeepmistry/arduino-LoRa
#include <Arduino_JSON.h>         // Biblioteca JSON https://github.com/arduino-libraries/Arduino_JSON
#include <TinyGPSPlus.h>          // Biblioteca GPS https://github.com/mikalhart/TinyGPSPlus
#include <SoftwareSerial.h>       // Biblioteca SoftwareSerial do Arduino necessária para a comunicação Serial com o módulo GPS
#include <RTClib.h>               // Biblioteca RTC https://github.com/adafruit/RTClib
#include "LowPower.h"             // Biblioteca LowPower https://github.com/LowPowerLab/LowPower


#define BAND 868E6                        // Frequência do rádio LoRa
#define SPREADING_FACTOR 7                // Spreading Factor do rádio LoRa
#define PREAMBLE_LENGTH 8                 // Preamble Length do rádio LoRa
#define BANDWIDTH 125E3                   // Bandwidth do rádio LoRa
#define CODING_RATE 7                     // Coding Rate do rádio LoRa
#define SYNC_WORD 0x12                    // Sync Word do rádio LoRa

#define devEUI "00:00:00:00:00:02:01:01"  // Endereço de identificação do dispositivo
#define boardName "LoRa Radio Node v1"    // Nome da placa utilizada
#define application "GPS Tracker"  // Nome da aplicação

#define CLOCK_INTERRUPT_PIN 3             // Pino em que SQW do RTC está conectado no qual o alarme vai ser disparado (Pino digital)
#define TIME_TO_SLEEP 10                  // Tempo que o dispositivo dorme em cada ciclo (Em segundos)
#define TIME_TO_SLEEP_LOST 3              // Tempo que o dispositivo dorme em cada ciclo casos esteja perdido (Em segundos)
#define RX_TIMEOUT 3                      // Tempo que o dispositivo vai estar em modo Rx

#define txSensorPin A0                    // Pino onde o Tx do Sensor está conectado
#define rxSensorPin A1                    // Pino onde o Rx do Sensor está conectado

typedef enum
{
  LOWPOWER,
  RECEIVE,
  SEND_REQUEST,
  SEND_LOCATION
}States_t;

States_t state;

int timeToSleep = TIME_TO_SLEEP;
RTC_DS3231 rtc;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(txSensorPin, rxSensorPin);

JSONVar receivedJson;
String receivedData;

bool flager = false;
bool isLost = false;
int rxTime;
bool rxFlag;


// Método de inicialização do rádio LoRa
void initLoRa(){
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


// Método que é chamado ao disparar o alarme do RTC
void onAlarm() {
    //Serial.println("Alarm occured!");
}

// Método de inicialização do RTC
void setupRTC(int interruptPin){

  // Inicialização do RTC
  if(!rtc.begin()) {
      Serial.println("Couldn't find RTC!");
      Serial.flush();
      while (1) delay(10);
  }

  if(rtc.lostPower()) {
      // Coloca a data de compilação no RTC
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Desativa o Pino que produz uma squarewave a 32KHz
  rtc.disable32K();

  // Inicializar o pin que dispara o alarme
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), onAlarm, FALLING);

  // Desativar as flags dos alarmes do RTC
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  // Desativar a produção de squarewave para poder usar o pino como trigger do alarme
  rtc.writeSqwPinMode(DS3231_OFF);

  // Desativar o alarme 1
  rtc.disableAlarm(1);

  //Serial.println("RTC initialized");
}

// Método que desativa o alarme
void clearAlarm(){
  if (rtc.alarmFired(2)) {
      rtc.clearAlarm(2);
      //Serial.print(" - Alarm cleared");
  }
}

// Método que programa o alarme para um intervalo de tempo
void setAlarm(int seconds){
  if(!rtc.setAlarm2(
            rtc.now() + TimeSpan(seconds),
            DS3231_A2_Hour // Modo do alarme 2 que dispara quando as horas e os minutos forem iguais
    )) {
        //Serial.println("Error, alarm wasn't set!");
    }else {
        /*Serial.print("\nAlarm will happen in ");
        Serial.print(seconds);
        Serial.print(" seconds!");*/
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
  Serial.println(json);
}

// Método que envia um request do estado do dispositivo
void sendStatusRequest(){
  JSONVar doc;
  doc["devEUI"] = devEUI;
  doc["application"] = application;
  doc["board"] = boardName;
  doc["data"]["status"] = "request";
  String json = JSON.stringify(doc);
  Serial.println(json);
  sendPacket(json);
}

// Método que envia a String num pacote LoRa
void sendPacket(String data){
  LoRa.beginPacket();
  LoRa.print(data);
  LoRa.endPacket();
}

// Método que é chamado ao acordar do dispositivo
void wakeUp(){
}

// Método que verifica o estado do dispositivo através do pacote recebido
void checkRequestReply(){
  JSONVar receivedJson;
  receivedJson = JSON.parse(receivedData);
  String destination = receivedJson["destination"];

  // Verifica que o pacote que recebeu é deste dispositivo
  if(destination == devEUI){
    isLost = receivedJson["data"]["status"]; 
  }
}

// Método que analiza o pacote recebido
void receivePacket(int packetSize){
  // Verifica se o pacote é vazio
  if(packetSize == 0){
    return;
  } 

  // Guarda o RSSi do pacote
  int rssi = LoRa.packetRssi();

  String data = "";

  // Verifica e guarda os bytes do pacote recebido
  while(LoRa.available()){
    byte b = LoRa.read();
    // Verifica se é um caracter válido
    if(32 <= b && b <= 127){
      data += (char)b;
    }
    // Verifica se o primeiro caracter do pacote é um {
    if(data[0] != '{')
      data = data.substring(1);
  }
  receivedData = data;
  flager = true;
}


// Método de inicialização do dispositivo
void setup() {
  Serial.begin(57600);                      // Inicialização da comunicação Serial
  initLoRa();                               // Inicialização do rádio LoRa
  pinMode(CLOCK_INTERRUPT_PIN, INPUT);      // Colocar o pino do alarme em modo Input
  setupRTC(CLOCK_INTERRUPT_PIN);            // Inicializar o módulo RTC

  state = SEND_REQUEST;
}

// Método de funcionamento do dispositivo
void loop() {
  switch (state){

    // Estado que envia o request do estado do dispositivo
    case SEND_REQUEST:
      Serial.println("Send Request");
      gpsSerial.begin(9600);
      sendStatusRequest();
      state = RECEIVE;
      break;

    // Estado que envia a localização do dispositivo
    case SEND_LOCATION:
      Serial.println("Send Location");
      delay(500);
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
      Serial.println("Going to sleep for " + String(timeToSleep/60) + " minutes");
      gpsSerial.end();  // Desliga a comunicação serial com o módulo GPS (Senão o dispostivo não entra em modo low power)
      setAlarm(timeToSleep);  // Programa o alarme para acordar o dispositivo
      attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), wakeUp, LOW); // Ativa a interrupção para quando o alarme disparar
      delay(100);
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);  // Coloca o dispositivo a dormir
      detachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN));  // Desativa a interrupção do alarne
      clearAlarm(); // Desativa o alarme
      state = SEND_REQUEST;
      break;
    default:
      state = LOWPOWER; 
      break; 
  }
}
