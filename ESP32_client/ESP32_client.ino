

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>

#include <WebSocketsClient.h>
#include <ArduinoJson.h>

#include <HTTPClient.h>
#include "mbedtls/base64.h"

WiFiMulti WiFiMulti;
WebSocketsClient webSocket;

#define checkConnectionWifiOk 26
#define checkConnectionServer 32
#define SendData 33

//=============3 chân tín hiệu từ máy đưa vâof
const int relayPin1 = 13; 
const int relayPin2 = 12;
const int relayPin3 = 14;
int relayState1 = HIGH;    // Trạng thái của relay (mặc định là HIGH)
int lastRelayState1 = HIGH; // Trạng thái trước đó của relay

int relayState2 = HIGH;    // Trạng thái của relay (mặc định là HIGH)
int lastRelayState2 = HIGH; // Trạng thái trước đó của relay

int relayState3 = HIGH;    // Trạng thái của relay (mặc định là HIGH)
int lastRelayState3 = HIGH; // Trạng thái trước đó của relay


//Lấy địa chỉ mac của thiết bị
bool macFound;

bool isAuthorized;  // Cờ dừng chương trình
int runDevice = 0;
bool isStopProgram=false;


unsigned long lastCheckTime = 0; // Lần cuối gửi request
unsigned long startTime;
int i=0;

void setupIO(){
  pinMode(relayPin1, INPUT_PULLUP);  // Cấu hình chân relayPin là đầu vào với điện trở kéo lên
  pinMode(relayPin2, INPUT_PULLUP);  // Cấu hình chân relayPin là đầu vào với điện trở kéo lên
  pinMode(relayPin3, INPUT_PULLUP);  // Cấu hình chân relayPin là đầu vào với điện trở kéo lên
  
  pinMode(checkConnectionWifiOk,OUTPUT);
  pinMode(checkConnectionServer,OUTPUT);
  pinMode(SendData,OUTPUT);
  digitalWrite(checkConnectionWifiOk,LOW);
  digitalWrite(checkConnectionServer,LOW);
  digitalWrite(SendData,LOW);
  
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


//
String base64Encode(String input) {
    size_t inputLen = input.length();
    size_t encodedLen = 0;

    // Xác định kích thước bộ đệm cần thiết để lưu dữ liệu sau khi mã hóa
    mbedtls_base64_encode(NULL, 0, &encodedLen, (const unsigned char*)input.c_str(), inputLen);

    // Tạo bộ đệm để lưu dữ liệu sau khi mã hóa
    unsigned char encodedOutput[encodedLen + 1]; // +1 để đảm bảo kết thúc bằng NULL

    // Thực hiện mã hóa Base64
    if (mbedtls_base64_encode(encodedOutput, encodedLen, &encodedLen, (const unsigned char*)input.c_str(), inputLen) == 0) {
        encodedOutput[encodedLen] = '\0';  // Đảm bảo chuỗi kết thúc bằng NULL
        return String((char*)encodedOutput);
    } else {
        return "Lỗi mã hóa!";
    }
}


String base64Decode(String input) {
    size_t decodedLen = 0;
    size_t inputLen = input.length();
    
    // Xác định kích thước bộ đệm cần thiết để lưu dữ liệu sau khi giải mã
    mbedtls_base64_decode(NULL, 0, &decodedLen, (const unsigned char*)input.c_str(), inputLen);

    // Tạo bộ đệm để lưu dữ liệu giải mã
    unsigned char decodedOutput[decodedLen + 1]; // +1 để thêm ký tự null

    // Thực hiện giải mã Base64
    if (mbedtls_base64_decode(decodedOutput, decodedLen, &decodedLen, (const unsigned char*)input.c_str(), inputLen) == 0) {
        decodedOutput[decodedLen] = '\0';  // Đảm bảo chuỗi kết thúc bằng NULL
        return String((char*)decodedOutput);
    } else {
        return "Lỗi giải mã!";
    }
}
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

  switch(type) {
    case WStype_CONNECTED:
            //Nếu kết nối server
            digitalWrite(checkConnectionServer,HIGH);
            Serial.println("✅ Connected to WebSocket Server!");
            break;
        case WStype_DISCONNECTED:
            digitalWrite(checkConnectionServer,LOW);
            Serial.println("❌ Disconnected from WebSocket Server!");
            break;
        case WStype_TEXT:
        {
            Serial.printf("📩 Received: %s\n", payload);
        }
            break;
    case WStype_BIN:
      Serial.printf("[WSc] get binary length: %u\n", length);
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

String getHardware(){
   //======================Lấy địa chỉ MAC của esp
   uint8_t mac[6];
   WiFi.macAddress(mac);

   char macStr[18];
   sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]); 
   return macStr;
}


bool checkHardware(String input){
  String url = base64Decode("aHR0cHM6Ly9naXRpbnRlcm5ldC5naXRodWIuaW8vRVNQMzIvbWFjLmpzb24=");
  if(url == input){
    return true;  
  }
  return false;
}

void checkMac(){
  String checkDecode = base64Decode("Jmh0dHBzOi8vZ2l0aW50ZXJuZXQuZ2l0aHViLmlvL0VTUDMyL21hYy5qc29uKkEwOkI3OjY1OjI5OjE1OkEwKDEp");
  int indexStart = checkDecode.indexOf("&");
  int indexEnd = checkDecode.indexOf("*");
  String urlDecode = checkDecode.substring(indexStart+1, indexEnd);
  bool checkLink =  checkHardware(urlDecode);
  String serverUrl="";
  if(checkLink == true){
    serverUrl = urlDecode;
  }
  else{
    return;
  }
  if (WiFi.status() == WL_CONNECTED) {
          HTTPClient http;
          http.begin(serverUrl);
          int httpResponseCode = http.GET();  // Gửi yêu cầu GET
          if (httpResponseCode > 0) {
              String payload = http.getString();  // Nhận phản hồi JSON
              DynamicJsonDocument doc(1024); // Kích thước bộ nhớ động
              DeserializationError error = deserializeJson(doc, payload); 
              if (error) {
                Serial.print("JSON Parsing failed: ");
                Serial.println(error.f_str());
                return;
               }  
              String macStr = getHardware();
               for (JsonObject device : doc.as<JsonArray>()) {
                   String macAddress = device["mac"].as<String>();
                   if (macAddress.equalsIgnoreCase(macStr)) {
                      macFound = true;
                      runDevice = device["run"].as<String>().toInt();                      
                      if(runDevice == 0){                      
                        isAuthorized = true;
                      }
                      else{
                        isAuthorized = false;
                      }
                  }
               }                    
          }
          //end http
          else {
              Serial.print("Error on HTTP request: ");
              Serial.println(httpResponseCode);
              macFound = false;
              isAuthorized = false;
          }
          http.end();         
  }
}
void setup() {
    Serial.begin(9600);
    setupIO();
    Serial.setDebugOutput(true);
      for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
        delay(1000);
      }
    //WiFiMulti.addAP("Guest", "Trianhpc@2025#1");
    //WiFiMulti.addAP("Ut", "tritam1615");
    WiFiMulti.addAP("DK2024", "0962115216");

    while(WiFiMulti.run() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");
        //----------------Nếu chưa kết nối wifi => báo ra đèn đỏ
        digitalWrite(checkConnectionWifiOk,LOW);
    }

    //-------------Đèn báo kết nối Wifi thành công
    digitalWrite(checkConnectionWifiOk,HIGH);
    Serial.println("Connected To Wifi");
    Serial.println( WiFi.localIP());

      // server address, port and URL
      //webSocket.begin("172.168.1.114", 8000, "/");
      //webSocket.begin("192.168.1.12", 8000, "/");
      webSocket.begin("192.168.1.62", 8000, "/");
      webSocket.onEvent(webSocketEvent);
      webSocket.setReconnectInterval(5000);
      checkMac();
      startTime = millis();     
}

void loop() {
      if (WiFi.status() == WL_CONNECTED){
          digitalWrite(checkConnectionWifiOk,HIGH);
          if (millis() - lastCheckTime >= 5000) {
              i++;
              lastCheckTime = millis();
          }
          //1 ngày
          if(i > 24){
             checkMac();
             i=0;
          }
          if(macFound == true && isAuthorized == true){
              webSocket.loop();
              StaticJsonDocument<200> jsonDoc;
              String macStr = getHardware();
              //----------------
              jsonDoc["mac"] = macStr;
              jsonDoc["id"] = "M1";
              jsonDoc["a1"] = "";
              relayState1 = digitalRead(relayPin1);
              jsonDoc["a1"] = relayState1;
              
              relayState2 = digitalRead(relayPin2);
              jsonDoc["a2"] = relayState2;
              relayState3 = digitalRead(relayPin3);
              jsonDoc["a2"] = relayState3;
              char jsonStr[256];
              serializeJson(jsonDoc, jsonStr);
              if (relayState1 != lastRelayState1) {
                //Có sự thay đổi trạng thái relay gửi data lên server
                    if (relayState1 == LOW) {
                        digitalWrite(SendData,HIGH);
                        webSocket.sendTXT(jsonStr);
                        digitalWrite(SendData,LOW);
                    } 
                    else {
                      digitalWrite(SendData,HIGH);
                      webSocket.sendTXT(jsonStr);
                      digitalWrite(SendData,LOW);
                    }
                    lastRelayState1 = relayState1;  // Cập nhật trạng thái
               }
               if (relayState2 != lastRelayState2) {
                //Có sự thay đổi trạng thái relay gửi data lên server
                    if (relayState2 == LOW) {
                        digitalWrite(SendData,HIGH);
                        webSocket.sendTXT(jsonStr);
                        digitalWrite(SendData,LOW);
                    } 
                    else {
                      digitalWrite(SendData,HIGH);
                      webSocket.sendTXT(jsonStr);
                      digitalWrite(SendData,LOW);
                    }
                    lastRelayState2 = relayState2;  // Cập nhật trạng thái
               }
              if (relayState3 != lastRelayState3) {
                //Có sự thay đổi trạng thái relay gửi data lên server
                    if (relayState3 == LOW) {
                        digitalWrite(SendData,HIGH);
                        webSocket.sendTXT(jsonStr);
                        digitalWrite(SendData,LOW);
                    } 
                    else {
                      digitalWrite(SendData,HIGH);
                      webSocket.sendTXT(jsonStr);
                      digitalWrite(SendData,LOW);
                    }
                    lastRelayState3 = relayState3;  // Cập nhật trạng thái
               }   
          }
          //2 năm
          if (i> 1051200) {
            Serial.println("Đã đến hạn bảo trì");
            while(true);  
          }
                     
          }
          ///=============================End Wifi connected
          else{
              Serial.println("Mất kết nối đến WiFi");
              digitalWrite(checkConnectionWifiOk,LOW);
           }   
}
