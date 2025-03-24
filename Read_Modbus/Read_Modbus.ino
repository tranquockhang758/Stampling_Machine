/*

  Basic.pde - example using ModbusMaster library

  Library:: ModbusMaster
  Author:: Doc Walker <4-20ma@wvfans.net>

  Copyright:: 2009-2016 Doc Walker

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/

#include <ModbusMaster.h>

#define MAX485_DE 8
// instantiate ModbusMaster object
ModbusMaster node;
int PLCDataArray[39] = {0};
void preTransmission() {
  digitalWrite(MAX485_DE, HIGH);
}

void postTransmission() {
  digitalWrite(MAX485_DE, LOW);
}
//==================================Setup RS485
void setupRS485(){
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, LOW);  // Set RS485 to receive mode
  Serial1.begin(9600);   
  //Sử dụng
  node.begin(1, Serial1);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  Serial.println("Modbus Master Initialized");
}


uint16_t* readDataModbus(int start,int numHolding){
  uint8_t result;
  static uint16_t arrData[39];
  
  // Read Holding Register 0x0001 (Address 1)
  result = node.readHoldingRegisters(start, numHolding);
  if (result == node.ku8MBSuccess) {
   for (int i = 0; i < numHolding; i++) {
      arrData[i] = node.getResponseBuffer(i);
    }
    return arrData;
  } else {
    if(result == 226){
      Serial.println("Error: Mất két nối với slave 1");
    }
    if(result == 224){
      Serial.println("Error: Chưa đúng cấu trúc Modbus Slave");
    }
    if(result == 2){
      Serial.println("Error: địa chỉ thanh ghi trên slave 1 bị sai");
    }
    else{
      Serial.print("Error: ");
      Serial.println(result);
    }
  }

}
void setup()
{
  Serial.begin(9600);
  setupRS485();
}


void loop()
{
  uint16_t* arrData1;
  uint16_t* arrData2;
  uint16_t* arrData3;
  uint16_t* arrData4;  uint8_t result;
  //0-9
  arrData1 = readDataModbus(0,10);
  for(int i=0; i < 10 ; i++){
    PLCDataArray[i] = arrData1[i];
    Serial.println(PLCDataArray[i]);
  }
  delay(10);
  //10 -19
  arrData2 = readDataModbus(10,10);
  for(int i=10; i < 20 ; i++){
    PLCDataArray[i] = arrData2[i];
    Serial.println(PLCDataArray[i]);
  }
  delay(10);
  //20 -29
  arrData3 = readDataModbus(20,10);
  for(int i=20; i < 30 ; i++){
    PLCDataArray[i] = arrData3[i];
    Serial.println(PLCDataArray[i]);
  }
  delay(10);
  //30 -39
  arrData4 = readDataModbus(30,9);
  for(int i=30; i < 39 ; i++){
    PLCDataArray[i] = arrData4[i];
    Serial.println(PLCDataArray[i]);
  }
//  delay(10);
  
}
