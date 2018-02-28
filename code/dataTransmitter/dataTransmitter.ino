// Library to control the SHT1x sensor.
#include <SHT1x.h>
#include <string.h>

// Library to control timer interrupt.
#include <TimerOne.h>


// Libraries to control downtime and energy savings.
#include <avr/sleep.h>
#include <avr/power.h>

// Library to Watchdog timer.
#include <avr/wdt.h>

//Library to control the NRF24L01 Module
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"


// Define the CE and CSN pins for the NRF24L01 module.
#define CE_PIN   7
#define CSN_PIN 10


#include "hackair.h"
// Specify your sensor
hackAIR sensor(SENSOR_PMS5003);


// Library to control BMP180.
#include <Wire.h>
#include <Adafruit_BMP085.h>

// Channel or 'pipe' Note: The "LL" at the end of the constant indicate that the type is "LongLong".
// const uint64_t pipe = 0xE8E8F0F0E1LL;
byte addresses[][6] = {"1Node", "2Node"};
// Initialize Radio.
RF24 radio(CE_PIN,CSN_PIN);

// Define the data pin and the clock pin for the sht1x module.
#define dataPin 6
#define clockPin 5

#define NUMBER_OF_PARAMETERS 8      // Number of sensors/parameters to measure.
const int n=NUMBER_OF_PARAMETERS;
float sensorData[n];

// Initialise SHT1x.
SHT1x sht1x(dataPin, clockPin);

// Initialise.
Adafruit_BMP085 bmp;

// Global variable declarations.
volatile int f_wdt = 1;
int counter = 0;
int packetCounter = 0;

// Global variables for rain meter
const int REED = 2;      // The reed switch outputs to digital pin 2
int val = 0;             // Current value of reed switch
int old_val = 0;         // Old value of reed switch
int REEDCOUNT = 0;       // The intial count is zero
volatile float depth_of_rain;
int dust;

float voltagePercent;

long previousMillis = 0;
long interval = 10000;

// counterHandler: Controls how long the controller should stay asleep
void counterHandler(){
    //Increment Counter.
    counter++;
    // If it controls the time the controller should stay asleep
    // 1: For tests
    // 75: 10 minutes (75 * 8 = 600 seconds = 10 minutes)

    if(counter == 1) {
        // Reset when counter expires.
        counter = 0;

        // Turn on Device.
        power_all_enable();

        // Turn on Radio.
        radio.powerUp();

        // Wait for the radio to start.
        delay(2);
    }
    else {
        // if time not fulfilled, continue sleep.
        enterSleep();
    }
}
ISR(WDT_vect){
    // Stop WDT
    f_wdt = 1;
}

// setupWDT: Configure WDT
void setupWDT(){
    // Configure WDT to interrupt every 8 sec.
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = 1<<WDP0 | 1<<WDP3;
    WDTCSR |= _BV(WDIE);
}

// enterSleep: Turns off radio and keeps the controller in energy saving
void enterSleep(){
    // Initialize WDT
    f_wdt = 0;
    // Turn off Radio.
    radio.powerDown();
    // controller energy saving mode.
    sleep_enable();
    sleep_mode();
    // Wake the controller.
    sleep_disable();
    // Increase the interrupt counter.
    counterHandler();
}

// setupRadio: Configure Radio.
void setupRadio(){
    radio.begin();    // Initialize radio.
    radio.setRetries(15,15);  // Define the number of retires.
    radio.setDataRate(RF24_250KBPS); // Set the bitrate.
    radio.setPALevel(RF24_PA_MIN);    // Defines the amplifier level of the radio module (RF24_PA_MIN for testing, RF24_PA_HIGH for long distances)
    radio.setChannel(124);    // Define a radio channel for broadcast(0-127).
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
    radio.enableDynamicPayloads(); // Enable Dynamic Payloads.
    radio.startListening();
  }

// Rain calculation in mm.
void RainMeter(){
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

void setup(){
  // Serial.begin(115200);
  Serial.println("Starting");
  sensor.begin();
  bmp.begin();
  pinMode(REED,INPUT);
  Timer1.initialize(100000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt(RainMeter); // attach the service routine here
  sleep_bod_disable();// Disable brownout Detection for low power consumption.
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);// Set sleep mode.
  setupWDT();// Set WDT.
  setupRadio();// Set Radio.
}

void loop(){
    struct hackAirData data;
    sensor.refresh(data);

    int64_t value;
    float voltage;
    unsigned char dataRx;

    sensorData[0] = 0;
    sensorData[1] = 0;
    sensorData[2] = 0;
    sensorData[3] = 0;
    sensorData[4] = 0;
    sensorData[5] = 0;
    sensorData[6] = 0;
    sensorData[7] = 0;

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

    if ( radio.available()) {
      while (radio.available()) {
        radio.read( &dataRx, sizeof(unsigned char));
        if (dataRx == '1'){
          sensorData[0] = sht1x.readTemperatureC();
          sensorData[1] = sht1x.readHumidity();
          sensorData[2] = data.pm25;
          sensorData[3] = bmp.readAltitude();
          sensorData[4] = bmp.readPressure()*(1e-2);
          sensorData[5] = bmp.readSealevelPressure()*(1e-2);
          sensorData[6] = depth_of_rain;
          sensorData[7] = voltagePercent;
          if (isnan(sensorData[0]) || isnan(sensorData[1])|| isnan(sensorData[2])|| isnan(sensorData[3])|| isnan(sensorData[4])|| isnan(sensorData[5])|| isnan(sensorData[6])|| isnan(sensorData[7])) {
              Serial.println("Failed to read sensor!!");
              return;
          }
          radio.stopListening();
          radio.write(sensorData, sizeof(sensorData));
        }
      }
    }
  radio.startListening();
  }
    //   unsigned long currentMillis = millis();
    //   if(currentMillis - previousMillis > interval) {
    //     previousMillis = currentMillis;
    //     radio.openWritingPipe(pipe);
    //     radio.write(sensorData, sizeof(sensorData));
    //     radio.startListening();
    //     unsigned long started_waiting_at = millis();
    // }
    // while ( ! radio.available() ) {
    //   // Oh dear, no response received within our timescale
    //   if (millis() - started_waiting_at > 200 ) {
    //     Serial.println("No response received - timeout!");
    //     return;
    //   }
    // }
    // unsigned char dataRx;
    // radio.read( &dataRx, sizeof(unsigned int));
    // interval = 1000*(dataRx);
