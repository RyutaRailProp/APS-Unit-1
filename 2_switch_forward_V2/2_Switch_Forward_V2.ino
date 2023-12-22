#include <SPI.h>
#include <stdio.h>
#include <string.h>
#include <Wire.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define spiDeviceNum 8

#define PIN_SENSE1 2 //where we connected the fan sense pin. Must be an interrupt capable pin (2 or 3 on Arduino Uno)
#define PIN_SENSE2 3
#define DEBOUNCE 2000 //0 is fine for most fans, crappy fans may require 10 or 20 to filter out noise
#define FANSTUCK_THRESHOLD 500000 //if no interrupts were received for 500ms, consider the fan as stuck and report 0 RPM

// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS A0
// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);	
// Pass oneWire reference to DallasTemperature library
/*
DallasTemperature sensors(&oneWire); 
int dallasTempDeviceCount = 0;
float tempC1;
*/

//PWM Stuff
void setupTimer1();
void setPWM1A(float f);
void setPWM1B(float f);
void tachISR1();
unsigned long calcRPM(unsigned long volatile tsA, unsigned long volatile tsB);
float getTemp(int pin);
uint32_t spiread32(void);

unsigned long volatile ts1=0, ts2=0, ts3=0, ts4=0;
float PWM_Ratio0 = 0.4;
float PWM_Ratio1 = 1;

//Thermocouple
int spiPinAssign[] = {4, 5, 6, 7, A2, A3, 2, 3}; //SPI pins
float thermTemp[6] = {0, 0, 0, 0, 0, 0}; //Temp arrays

//RTC
RTC_DS1307 rtc;

//Misc
unsigned long prevTime;
char serialDataBuffer[80] = "\0";
long index = 33;
int controlTemp = thermTemp[4];
String LVR = "F";
String OTemp = "F";

void setup() {
  //sensors.begin();	// Start up the library
  Serial.begin(9600);

  //dallasTempDeviceCount = sensors.getDeviceCount();
  //Serial.println(dallasTempDeviceCount, DEC);
  
  //SPI Settings
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
  SPI.begin();
  
  // This line sets the RTC with an explicit date & time
  /*
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
  */

  //rtc.adjust(DateTime(2023, 11, 7, 14, 50, 0));

  // Set CS of SPI devices based on spiPinAssign array
  for (int i = 0; i < spiDeviceNum; i++){
    pinMode(spiPinAssign[i], OUTPUT);
    digitalWrite(spiPinAssign[i], HIGH);
  }

  //LVR PIn
  pinMode(8, INPUT_PULLUP);

  //Controls On Board Relay at OTemp Event
  pinMode(A1, OUTPUT);
  digitalWrite(A1, LOW);

  //this section sets up PWM pins 9 & 10, 1A & 1B respectively
  pinMode(9,OUTPUT); //1A
  pinMode(10,OUTPUT); //1B
  setupTimer1();

  setPWM1A(PWM_Ratio0); //set duty % on pin 9
  setPWM1B(PWM_Ratio1); //set duty % on pin 10

  //Fan Encoder set up
  pinMode(PIN_SENSE1,INPUT_PULLUP); //set the sense pin as input with pullup resistor
  pinMode(PIN_SENSE2,INPUT_PULLUP); //set the sense pin as input with pullup resistor
  //attachInterrupt(digitalPinToInterrupt(PIN_SENSE1),tachISR1,FALLING); //set tachISR to be triggered when the signal on the sense pin goes low
  //attachInterrupt(digitalPinToInterrupt(PIN_SENSE2),tachISR2,FALLING); //set tachISR to be triggered when the signal on the sense pin goes low]

  /*
  //Real Time Clock
  DateTime now = rtc.now();

  strcpy(serialDataBuffer, String(now.year()).c_str());
  strcat(serialDataBuffer, "\r");
  strcat(serialDataBuffer, String(now.month()).c_str());
  strcat(serialDataBuffer, "\r");
  strcat(serialDataBuffer, String(now.day()).c_str());
  strcat(serialDataBuffer, "\r");
  strcat(serialDataBuffer, String(now.hour()).c_str());
  strcat(serialDataBuffer, "\r");
  strcat(serialDataBuffer, String(now.minute()).c_str());
  strcat(serialDataBuffer, "\r");
  strcat(serialDataBuffer, String(now.second()).c_str());
  strcat(serialDataBuffer, "\n");
  Serial.write(serialDataBuffer);
  */

  delay(5000);
}

void loop() {
  //LVR
  if (digitalRead(8) == HIGH) {
    LVR = "F";
  } else {
    LVR = "T";
  }

  //check for broken thermocouple 
  if (thermTemp[4] == 999 & thermTemp[5] != 999) {
    controlTemp = thermTemp[5];
  } else if (thermTemp[5] == 999 & thermTemp[4] != 999) {
    controlTemp = thermTemp[4];
  } else if (thermTemp[4] == 999 & thermTemp[5] == 999) {
    controlTemp = 0;
  } else {
    controlTemp = thermTemp[4];
  }

  //OTemp
  if (999 > controlTemp > 150) {
    digitalWrite(A1, HIGH);
    OTemp = "T";
  } else {
    digitalWrite(A1, LOW);
    OTemp = "F";
  }

  /*
  //Fan Control Loop
  PWM_Ratio0 = 1 - ((100/35.0) * controlTemp - 70) / 100;
  if (PWM_Ratio0 > 1) {
    setPWM1A(1);
  } else if (PWM_Ratio0 < 0) {
    setPWM1A(0);
  } else {
    setPWM1A(PWM_Ratio0);
  }
  */

  //Temperature Analog Output
  PWM_Ratio1 = 1 - ((100/120) * controlTemp + 16) / 100;
  if (PWM_Ratio1 > 1) {
    setPWM1B(1);
  } else if (PWM_Ratio1 < 0) {
    setPWM1B(0);
  } else {
    setPWM1B(PWM_Ratio1);
  }


  //Serial Transmit Loop
  if (millis() - prevTime >= 1000) {
    //Serial.println(millis());
    //sensors.requestTemperatures();
    char serialDataBuffer[80] = "\0";

    //Index
    strcpy(serialDataBuffer, String(char(index)).c_str());
    strcat(serialDataBuffer, "\r");
    index++;

    if (index > 255) {
      index = 33;
    }

    //Thermocouple Data
    for (int i = 0; i < spiDeviceNum; i++) {
      thermTemp[i] = getTemp(spiPinAssign[i]);

      if (thermTemp[i] >= 2047) {
        thermTemp[i] = 999;
      }
      // Serial.println(thermTemp);
      strcat(serialDataBuffer, String(thermTemp[i] * 10, 0).c_str());
      strcat(serialDataBuffer, "\r");  // concatonate CR
    }
    
    /*
    //Dallas OneWire Temp Sensor
    for (int i = 0; i < dallasTempDeviceCount; i++) {
      tempC1 = sensors.getTempCByIndex(i);
      strcat(serialDataBuffer, String(tempC1 * 10, 0).c_str());
      strcat(serialDataBuffer, "\r");  // concatonate CR
    }
    */
  
    for (int i = 0; i < 2; i++) {
      strcat(serialDataBuffer, "0");
      strcat(serialDataBuffer, "\r");  // concatonate CR
    }
  
    
    //Input Voltage
    float voltage = 750 * (analogRead(A7)/1023.0);
    strcat(serialDataBuffer, String(voltage, 0).c_str());
    strcat(serialDataBuffer, "\r");

    //Fan RPMs
    strcat(serialDataBuffer, String("0").c_str());
    strcat(serialDataBuffer, "\r"); 
    strcat(serialDataBuffer, String("0").c_str());
    strcat(serialDataBuffer, "\r"); 
    
    //Low Voltage Relay State Boolean
    strcat(serialDataBuffer, LVR.c_str());
    strcat(serialDataBuffer, "\r");  

    //OTemp State Boolean
    strcat(serialDataBuffer, OTemp.c_str());
    strcat(serialDataBuffer, "\n"); 

    Serial.write(serialDataBuffer);  
    prevTime = millis();
    //Serial.println(millis());
  }
}

// Get SPI data from MAX31855 chip serial output
uint32_t spiread32(void) {
  uint32_t v;
  v = SPI.transfer(0);
  v <<= 8;
  v |= SPI.transfer(0);
  v <<= 8;
  v |= SPI.transfer(0);
  v <<= 8;
  v |= SPI.transfer(0);
  return v;
}

// Get Thermocouple Temp in Celcius
float getTemp(int pin) {
  digitalWrite(pin, LOW); 
  uint32_t data = spiread32();
  if (data & 0x80000000) {
    // Negative value, drop the lower 18 bits and explicitly extend sign bits.
    data = 0xFFFFC000 | ((data >> 18) & 0x00003FFF);
  } else {
    // Positive value, just drop the lower 18 bits.
    data >>= 18;
  }
  digitalWrite(pin, HIGH);
  // Format Data to Celcius
  float centigrade = data * 0.25;
  return centigrade;
}


void setupTimer1() {
    //Set PWM frequency to about 25khz on pins 9,10 (timer 1 mode 10, no prescale, count to 320)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << CS10) | (1 << WGM13);
    ICR1 = 320;
    OCR1A = 0;
    OCR1B = 0;
}

//equivalent of analogWrite on pin 9
void setPWM1A(float f) {
    f=f<0?0:f>1?1:f;
    OCR1A = (uint16_t)(320*f);
}

//equivalent of analogWrite on pin 10
void setPWM1B(float f) {
    f=f<0?0:f>1?1:f;
    OCR1B = (uint16_t)(320*f);
}

/*
//Interrupt handler. Stores the timestamps of the last 2 interrupts and handles debouncing
void tachISR1() {
    unsigned long m=micros();
    if((m-ts2)>DEBOUNCE){
        ts1=ts2;
        ts2=m;
    }
}

void tachISR2() {
    unsigned long m=micros();
    if((m-ts4)>DEBOUNCE){
        ts3=ts4;
        ts4=m;
    }
}

//Calculates the RPM based on the timestamps of the last 2 interrupts. Can be called at any time.
unsigned long calcRPM(unsigned long volatile tsA, unsigned long volatile tsB){
    if(micros()-tsB<FANSTUCK_THRESHOLD&&tsB!=0){
        return (60000000.0/(tsB-tsA))/2.0;
    }else return 0;
}
*/
