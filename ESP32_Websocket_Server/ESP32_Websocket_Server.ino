#include <Arduino.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>

#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>  
#include <ModbusRtu.h>  

WiFiMulti WiFiMulti;
//-------------------------------Khai báo port của server là 8000
//AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(8000);


//----------------Khai báo biến chứa dữ liệu modbus
uint16_t au16data[39] ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//ESP32 đóng vài trò là slave với ID là 1 cổng giao tiếp là TX2 RX2
Modbus slave(2, Serial2, 0);
//----------------Khai báo biến chứa dữ liệu modbus


#define checkConnectionWifiOk 13
#define checkConnectionClient 14
#define Getdata 27
void setupIO(){
  pinMode(checkConnectionWifiOk,OUTPUT);
  pinMode(checkConnectionClient,OUTPUT);
  pinMode(Getdata,OUTPUT);
  digitalWrite(checkConnectionWifiOk,LOW);
  digitalWrite(checkConnectionClient,LOW);
  digitalWrite(Getdata,LOW);
  
}

void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
  const uint8_t* src = (const uint8_t*) mem;
  Serial.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
  for(uint32_t i = 0; i < len; i++) {
    if(i % cols == 0) {
      Serial.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
    }
    Serial.printf("%02X ", *src);
    src++;
  }
  Serial.printf("\n");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
                webSocket.sendTXT(num, "Connected");
            }
            break;
        case WStype_TEXT:
            {
                Serial.printf("[Client %u] Sent message: %s\n", num, payload);
               
                String message = (const char*) payload;
                webSocket.sendTXT(num,message.c_str());
                
                StaticJsonDocument<200> doc;
                DeserializationError error = deserializeJson(doc, payload);
                //Phân biệt từng thiết bị
                if (error) {
                    Serial.print("JSON Parsing failed: ");
                    Serial.println(error.c_str());
                    return;
                }
                if (doc.containsKey("mac") && doc.containsKey("id")) {
                    String mac = doc["mac"];
                    String id = doc["id"];
                    int dataRelay1,dataRelay2,dataRelay3;
                    if(mac == "A0:B7:65:29:15:A0" && id == "M1"){
                      String a1 = (doc["a1"]);
                      dataRelay1  = a1.toInt();
                      String a2 = (doc["a2"]);
                      dataRelay2  = a2.toInt();
                      String a3 = (doc["a3"]);
                      dataRelay3  = a3.toInt();
                      au16data[0] = dataRelay1;
                      au16data[1] = 100;
                      for(int i=0;i<3;i++){
                        Serial.println(au16data[i]);
                      }
                    }
                }

            }
                break;
        case WStype_BIN:
            Serial.printf("[%u] get binary length: %u\n", num, length);
            hexdump(payload, length);
            break;
  case WStype_ERROR:      
  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_FRAGMENT_FIN:
      break;
    }

}

void setup() {

    Serial.begin(9600);
    setupIO();
    Serial2.begin(9600,SERIAL_8N1,16,17);
    slave.begin(9600);
    Serial.setDebugOutput(true);
    Serial.println();
    Serial.println();
    Serial.println();

    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BO OT WAIT %d...\n", t);
        Serial.flush();
        delay(1000);
    }

    //WiFiMulti.addAP("DK2024", "0962115216");
    WiFiMulti.addAP("Guest", "Trianhpc@2025#1");
    //WiFiMulti.addAP("Ut", "tritam1615");


    while(WiFiMulti.run() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");
        
        //-----------------Nếu chưa kết nối được WIFI => Out ra đèn đỏ
        digitalWrite(checkConnectionWifiOk,LOW);
    }
    digitalWrite(checkConnectionWifiOk,HIGH);
    Serial.println("Connected To Wifi");
    Serial.println(WiFi.localIP());
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}

void loop() {
    slave.poll(au16data,39);
    if(WiFiMulti.run() == WL_CONNECTED){
      //Kiểm tra nếu còn kết nối ta thực hiện trao đổi dữ liệu client
        digitalWrite(checkConnectionWifiOk,HIGH);
        webSocket.loop();
    }
    else{
      digitalWrite(checkConnectionWifiOk,LOW);
    }
    
}
