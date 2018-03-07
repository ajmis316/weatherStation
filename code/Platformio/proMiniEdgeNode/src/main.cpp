#include <Arduino.h>

/*
* Getting Started example sketch for nRF24L01+ radios
* This is a very basic example of how to send data from one node to another
* Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"

#include <SHT1x.h>
#include "hackair.h"

#include <Adafruit_BMP085.h>

#include <TimerOne.h>


/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 0;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(7,10);
hackAIR sensor(SENSOR_PMS5003);
Adafruit_BMP085 bmp;

/**********************************************************/

byte addresses[][6] = {"1Node","2Node"};

// Used to control whether this node is sending or receiving
bool role = 0;

#define NUMBER_OF_PARAMETERS 8      // Number of sensors/parameters to measure.
const int n=NUMBER_OF_PARAMETERS;
float sensorData[n];
float voltagePercent;

const int REED = 2;      // The reed switch outputs to digital pin 2
int val = 0;             // Current value of reed switch
int old_val = 0;         // Old value of reed switch
int REEDCOUNT = 0;       // The intial count is zero
volatile float depth_of_rain;

#define dataPin 6
#define clockPin 5
SHT1x sht1x(dataPin, clockPin);

void measure(){

  int64_t value;
  float voltage;
  int num = 0;
  while (num< 1){
    int sensorValue = analogRead(A7);
    value = value + sensorValue;
    num++;
  }

  num=0;
  voltage  = ((value/1)*3.3)/1024;
  voltage = voltage * 5.44444;
  //voltagePercent = (voltage/12.6) * 100;
  if (voltage >= 11.1){
    voltagePercent = ((voltage-11.1)/1.5)*100;
  }
  float volume_of_rain;
  float area_of_funnel;
  // read REED pin.
  val = digitalRead(REED);
  //Serial.println("reading reed pin");

  if ((val == LOW) && (old_val == HIGH)){
    REEDCOUNT = REEDCOUNT + 1;
    delay(10);
    old_val = val;
    Serial.println(REEDCOUNT);
    }else {
      old_val = val;
    }
    volume_of_rain = (5.94*REEDCOUNT)/1000;
    area_of_funnel = (3.14/4)*0.165*0.165;
    depth_of_rain = volume_of_rain/area_of_funnel;
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("RF24/examples/GettingStarted"));
  // Serial.println(F("***PRESS 'T' to begin transmitting to the other node"));

  Timer1.initialize(100000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt(measure); // attach the service routine here

  radio.begin();
  sensor.begin();
  bmp.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);

  // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }

  // Start the radio listening for data
  radio.startListening();
}

void loop() {

  struct hackAirData data;
  sensor.refresh(data);
  // measure();

  sensorData[0] = sht1x.readTemperatureC();
  sensorData[1] = sht1x.readHumidity();
  sensorData[2] = data.pm25;
  sensorData[3] = bmp.readAltitude();
  sensorData[4] = bmp.readPressure()*(1e-2);
  sensorData[5] = bmp.readSealevelPressure()*(1e-2);
  sensorData[6] = voltagePercent;
  sensorData[7] = depth_of_rain;
/****************** Pong Back Role ***************************/

  if ( role == 0 )
  {
    unsigned long got_time;

    if( radio.available()){
                                                                    // Variable for the received timestamp
      while (radio.available()) {                                   // While there is data ready
        radio.read( &got_time, sizeof(unsigned long) );             // Get the payload
      }
      Serial.println(got_time);
      if (got_time == 102){
        radio.stopListening();                                        // First, stop listening so we can talk
        radio.write( &sensorData, sizeof(sensorData) );              // Send the final one back.
        radio.startListening();                                       // Now, resume listening so we catch the next packets.
        Serial.print(F("Sent response :"));
        Serial.println(sensorData[7]);
        // role = 2;
      }
   }
 }
} // Loop
