/* 
  Slaver/Receiver
  Baudrate  : 9600 - AT+IPR=9600
  Address   : 2    - AT+ADDRESS=2
  NETWORKID : 18   - AT+NETWORKID=18
  
  Set RF Frequency: AT+BAND=915000000 (default)
  
  *** Library *****
- LowPower        : https://github.com/rocketscream/Low-Power
- Adafruit_Sensor : https://github.com/adafruit/Adafruit_Sensor
- DHT             : https://github.com/adafruit/DHT-sensor-library

*/
// xin chao pro
//=========== Including the libraries.
#include <Arduino.h>
#include "LowPower.h"    
#include <SoftwareSerial.h>
#include <Wire.h> 
#include "Adafruit_Sensor.h"
#include "DHT.h"
#define DHTPIN 6     // what digital pin we're connected to
#define LEDPIN 7     // what digital pin we're connected t
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);

#define MoisturePin A0
#define VoltagePin A1

float  
VBAT = 0.0000,
VSEN = 0.0000; 
uint8_t avgCount = 2;

// Giá trị độ ẩm đất
uint16_t MoistureValue = 0;
const uint16_t MoistureValueMin = 239;   // wet
const uint16_t MoistureValueMax = 700;   // dry
// Giá trị điện áp pin
float Vref = 5.33;
float VoltageValue = 0;
uint8_t BateryPercent = 0;
// Giá trị độ ẩm không khí
uint8_t HumidityValue    = 0;
uint8_t TemperatureValue = 0;

// Blink LED báo trạng thái
#define LEDPIN 7    // Chân của LED
int ledState = LOW;  // Trạng thái của LED (ban đầu tắt)

// Defines the slave/destination address.
#define slave_Address   02
#define master_Address  01

SoftwareSerial ReyaxLoRa(5, 4); //--> RYLR998_RX, RYLR998_TX

void ReadMoistureValue()
{
  VSEN = 0.000;
  for(int i = 0; i<avgCount; i++){
    VSEN = VSEN + analogRead(MoisturePin);;
  }
  uint16_t RawValue = (VSEN/avgCount);
  if(RawValue < MoistureValueMin) RawValue = MoistureValueMin;
  if(RawValue > MoistureValueMax) RawValue = MoistureValueMax;
  MoistureValue = map(RawValue, MoistureValueMin, MoistureValueMax, 100, 0);
}

void ReadVoltageValue()
{
  VBAT = 0.000;
  for(int i = 0; i<avgCount; i++){
    VBAT = VBAT + analogRead(VoltagePin);;
  }
  float RawValue = (VBAT/avgCount); 
  VoltageValue = (RawValue*Vref)/1024.0;
  uint16_t IntVoltageValue = VoltageValue*100;  // 3.7*100 = 370
  if(IntVoltageValue < 350.0) IntVoltageValue = 350.0;
  if(IntVoltageValue > 420.0) IntVoltageValue = 420.0;
  BateryPercent = map(IntVoltageValue, 350.0, 420.0, 0, 100);
}

void ReadDHT11()
{
  HumidityValue = dht.readHumidity();
  TemperatureValue = dht.readTemperature();
}

//_______________ getValue()_______________ 
// String function to split data based on certain characters.
// Reference : https://www.electroniclinic.com/reyax-lora-based-multiple-sensors-monitoring-using-arduino/
// String getValue(String data, char separator, int index) {
//   int found = 0;
//   int strIndex[] = { 0, -1 };
//   int maxIndex = data.length() - 1;
  
//   for (int i = 0; i <= maxIndex && found <= index; i++) {
//     if (data.charAt(i) == separator || i == maxIndex) {
//       found++;
//       strIndex[0] = strIndex[1] + 1;
//       strIndex[1] = (i == maxIndex) ? i+1 : i;
//     }
//   }
//   return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
// }

// void Serial_Print()
// { 
//   Serial.print("VoltageValue: ");       Serial.print(VoltageValue);
//   Serial.print(" - MoistureValue: ");   Serial.print(MoistureValue);
//   Serial.print(F(" - Humidity: "));     Serial.print(HumidityValue);
//   Serial.print(F("% - Temperature: ")); Serial.print(TemperatureValue);
//   Serial.print("- BateryPercent: ");    Serial.println(BateryPercent);
//   delay(500);
// }

//________ReyaxLoRa_Send()_________
// Subroutine for sending data.
void ReyaxLoRa_Send(int addr, String data_send) {
  String str_Send;
  // AT commands to transmit data. 
  // For more details see the document "LoRa_AT_Command_RYLR998_RYLR498_EN.pdf" in the "AT+SEND" section.
  str_Send = "AT+SEND=" + String(addr) + "," + String(data_send.length()) + "," + data_send + "\r\n";
  ReyaxLoRa.print(str_Send);
  Serial.println();
  Serial.print("Send to Receiver : ");
  Serial.print(str_Send);
  Serial.flush();
}

//__________ ReyaxLoRa_Receive()__________
// Subroutine to receive data.
// void ReyaxLoRa_Receive() {
//   if (ReyaxLoRa.available() > 0 ) {
//     String rcv_Data_String = ReyaxLoRa.readString(); 
    
//     if(rcv_Data_String.indexOf("OK") > 0 || rcv_Data_String.indexOf("ERR") > 0) {
//       Serial.println();
//       Serial.print(F("LoRa Reyax Module Response : "));
//       Serial.println(rcv_Data_String);
//       return;
//     } else {
//       // Print received data or messages.
//       Serial.println(F("Received from Sender : "));
//       Serial.println(rcv_Data_String);
//       Serial.flush();

//       //---------------------------------------- Process incoming data.
//       // For more details see the document "LoRa_AT_Command_RYLR998_RYLR498_EN.pdf" in the section "+RCV".
      
//       //String _addr    = getValue(rcv_Data_String, ',', 0);    //--> address
//       //_addr = _addr.substring(5);
//       //String _length  = getValue(rcv_Data_String, ',', 1);    //--> data length
//       String _message = getValue(rcv_Data_String, ',', 2);    //--> data/message
//       //String _rssi    = getValue(rcv_Data_String, ',', 3);    //--> RSSI
//       //String _snr     = getValue(rcv_Data_String, ',', 4);    //--> SNR
  
//       //Serial.println();
//       //Serial.println(F("Received from Master."));
//       //Serial.print(F("-Addr     : "));
//       //Serial.println(_addr);
//       //Serial.print(F("-Length   : "));
//       //Serial.println(_length);
//       //Serial.print(F("-Message  : "));
//       //Serial.println(_message);
//       //Serial.print(F("-RSSI     : "));
//       //Serial.println(_rssi);
//       //Serial.print(F("-SNR      : "));
//       //Serial.println(_snr);
//       //Serial.flush();

//       ReadDHT11();
//       ReadMoistureValue();
//       ReadVoltageValue();
//       Serial_Print();
  
//       delay(100);

//       // Sends the state of LED_1 and LED_2 to the master.
//       ReyaxLoRa_Send(master_Address, String(TemperatureValue) + "|" + String(HumidityValue) + "|" + String(MoistureValue) + "|" + String(BateryPercent));
//     }
//   }
// }

void LoRaSleepMode()
{
  ReyaxLoRa.print("AT+MODE=1\r\n");  // Sleep Mode
  delay(300);
}

void LoRaTransceiverMode()
{
  ReyaxLoRa.print("AT+MODE=0\r\n");  // Transceiver mode
  delay(300);
}

void setup()
{
  Serial.begin(9600);
  ReyaxLoRa.begin(9600);
  pinMode(LEDPIN, OUTPUT);
  // khởi tạo DHT
  dht.begin();
  LoRaSleepMode();
}

void loop()
{ 
  LoRaTransceiverMode();  // Chuyển sang chế độ Transceiver
  ReadDHT11();
  ReadMoistureValue();
  ReadVoltageValue();
  delay(100);

  ReyaxLoRa_Send(master_Address, String(TemperatureValue) + "|" + String(HumidityValue) + "|" + String(MoistureValue) + "|" + String(BateryPercent));
  delay(500);
  LoRaSleepMode();       // Chuyển sang chế độ Sleep                  
  //LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF); // sleep 8s
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  

  //Serial_Print(); // debug
}