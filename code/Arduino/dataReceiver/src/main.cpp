#include <Arduino.h>

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <TimerOne.h>

#define CE_PIN   9
#define CSN_PIN 10

byte addresses[][6] = {"1Node", "2Node"};

float sensordata[8];
int data = 5;


RF24 radio(CE_PIN, CSN_PIN);

int packetCounter = 0;

void setupRadio()
{
  radio.begin();
  radio.setRetries(15, 15);
  radio.setChannel(124);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.enableDynamicPayloads();
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
}

void serialRead(){
 if (Serial.available()>0){
   String datas = Serial.readString();
//   data = datas.toInt();
   Serial.print(datas);
 }
}

void setup() {
  Serial.begin(57600);
  Serial.println("start");
  setupRadio();
  radio.powerDown();
  Timer1.initialize(100000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt(serialRead); // attach the service routine here
}

void loop() {
  sensordata[0] = 0;
  sensordata[1] = 0;
  sensordata[2] = 0;
  sensordata[3] = 0;
  sensordata[4] = 0;
  sensordata[5] = 0;
  sensordata[6] = 0;
  sensordata[7] = 0;

  if (Serial.available()>0){
    char data = Serial.read();
    if (data=='1'){
      Serial.print("one");
      radio.powerUp();
      radio.stopListening();
      radio.write( &data, sizeof(char));
      radio.startListening();
      if ( radio.available()) {
        while (radio.available()) {
          radio.read( &sensordata, sizeof(sensordata));
        }
        radio.stopListening();
        radio.powerDown();
      }
    }
  }
    Serial.print (sensordata[0]);
    Serial.print(',');
    Serial.print(sensordata[1]);
    Serial.print(',');
    Serial.print (sensordata[2]);
    Serial.print(',');
    Serial.print (sensordata[3]);
    Serial.print(',');
    Serial.print (sensordata[4]);
    Serial.print(',');
    Serial.print (sensordata[5]);
    Serial.print(',');
    Serial.print(sensordata[6]);
    Serial.print(',');
    Serial.println(sensordata[7]);
    Serial.print("sent data:");
    Serial.println(data);
}
