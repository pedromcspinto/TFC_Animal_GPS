  #include "LoRa_APP.h"
  #include "Arduino.h"
  #include <ArduinoJson.h>
  #include <TinyGPSPlus.h>

  #ifndef LoraWan_RGB
  #define LoraWan_RGB 0
  #endif

  #define RF_FREQUENCY                                868000000 // Hz

  #define TX_OUTPUT_POWER                             5        // dBm

  #define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                                //  1: 250 kHz,
                                                                //  2: 500 kHz,
                                                                //  3: Reserved]
  #define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
  #define LORA_CODINGRATE                             1         // [1: 4/5,
                                                                //  2: 4/6,
                                                                //  3: 4/7,
                                                                //  4: 4/8]
  #define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
  #define LORA_SYMBOL_TIMEOUT                         0         // Symbols
  #define LORA_FIX_LENGTH_PAYLOAD_ON                  false
  #define LORA_IQ_INVERSION_ON                        false


  #define RX_TIMEOUT_VALUE                            1000
  #define BUFFER_SIZE                                 250 // Define the payload size here
  #define timetillwakeup                              5000
  #define devEUI                                      "00:00:00:00:00:00:00:01"


  typedef enum
  {
    LOWPOWER,
    RX,
    SEND_REQUEST,
    SEND_LOCATION
  }States_t;
  States_t state;

  static TimerEvent_t wakeUp;

  char txpacket[BUFFER_SIZE];
  char rxpacket[BUFFER_SIZE];
  char jsonChArray[BUFFER_SIZE];

  static RadioEvents_t RadioEvents;
  void OnTxDone( void );
  void OnTxTimeout( void );
  void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
  void jsonToCharArray(StaticJsonDocument<BUFFER_SIZE> doc);

  StaticJsonDocument<BUFFER_SIZE> receivedJson;
  int16_t txNumber;
  int16_t Rssi,rxSize;
  TinyGPSPlus gps;
  bool isLost = false;

  void sendJsonData(){
    StaticJsonDocument<250> doc;
    doc["devEUI"] = devEUI;
    doc["application"] = "animal gps";
    doc["board"] = "CubeCell 1/2AA Node (HTCC-AB02A)";
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
    jsonToCharArray(doc);
    sendPacket(jsonChArray);
  }

  void sendStatusRequest(){
    StaticJsonDocument<250> doc;
    doc["devEUI"] = devEUI;
    doc["application"] = "animal gps";
    doc["board"] = "CubeCell 1/2AA Node (HTCC-AB02A)";
    doc["data"]["status"] = "request";
    jsonToCharArray(doc);
    sendPacket(jsonChArray);
  }

  void jsonToCharArray(StaticJsonDocument<BUFFER_SIZE> doc){
    String json;
    serializeJson(doc, json);
    json.toCharArray(jsonChArray, BUFFER_SIZE);
  }

  void sendPacket(char* data){
    txNumber++;
    Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",data, strlen(data));
    Radio.Send( (uint8_t *)data, strlen(data) );
  }

  void stringToJson(String data){
   
   String receivedPacket = String(data);
   receivedPacket = receivedPacket.substring(1,receivedPacket.length());
   DeserializationError error = deserializeJson(receivedJson, receivedPacket);

   if (error) {
     Serial.print(F("deserializeJson() failed: "));
     Serial.println(error.f_str());
     return;
   }
  }
      
void sleep(){
  TimerSetValue( &wakeUp, timetillwakeup );
  TimerStart( &wakeUp );
  lowPowerHandler();
}

void onWakeUp(){
  state = SEND_REQUEST;
}

void checkRequestReply(){
  if(receivedJson["destination"] == devEUI)
    isLost = receivedJson["data"]["status"]; 
}

  void setup() {
      
      Serial.begin(115200);
      Serial1.begin(9600);
      txNumber=0;
      Rssi=0;

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
    TimerInit( &wakeUp, onWakeUp );
    state = SEND_REQUEST;                                  
  }

  void loop()
  {
    switch (state){
      case SEND_REQUEST:
        sendStatusRequest();
        state = RX;
        break;
      case SEND_LOCATION:
        sendJsonData();  
        break;
      case RX:
        Radio.Rx(0);
        checkRequestReply();
        if(isLost)
          state = SEND_LOCATION;
        else
          state = LOWPOWER;
        break;
      case LOWPOWER:
          sleep();
        break;
      default: 
        break; 
    }
    delay(1000);
  }

  void OnTxDone( void ){}
  void OnTxTimeout( void ){
      Radio.Sleep( );
  }
  void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ){
      Rssi=rssi;
      rxSize=size;
      memcpy(rxpacket, payload, size );
      rxpacket[size]='\0';
      Radio.Sleep( );
      stringToJson(String(rxpacket));
      Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n",rxpacket,Rssi,rxSize);
      delay(1000);
  }