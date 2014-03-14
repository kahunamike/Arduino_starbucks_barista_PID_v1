#include <SoftwareSerial.h>
#include <serLCD.h>
#include <PID_v1.h>

//TempSensor: Voltage Reference and analog sensor pin
#define ANALOG_VOLTAGE_REFERENCE 5
#define TEMP_SENSOR_PIN 1

//LCD Display: Attach the serial display's RX line to digital pin 2
serLCD mySerial(2); // pin 2 = TX, pin 3 = RX (unused)
float TEMP_PRINT_INTERVAL = 500; //refresh screen every 500ms
float lastTempPrint=0;
int brew_status_num = 0; //0: heat, 1: ready, 2: hot

//PID Stuff
#define RelayPin 7
//PID settings: Define Variables we'll be connecting to
double Input, Output, Setpoint;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 500, 0.02, 6, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;

void setup() {
  Serial.begin(9600); //Serial Monitor
  mySerial.begin(9600); // Serial LCD: set up serial port for 9600 baud
  delay(1000); // wait for display to boot up

  //PID stuff
  windowStartTime = millis();
  pinMode(RelayPin, OUTPUT);

  //initialize the variables we're linked to
  //Water temp for espresso should be: 92-96C
  //Temperature sensor measuring on the outside of the boiler in C
  //set@96C (204F) and was reading 168F out of the brewhead
  Setpoint = 100;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  mySerial.clear();
  mySerial.selectLine(1);
  mySerial.print("Its-esPresso!");
  
}

void loop() {

  float currentTemp = getTemperature();
  //float volt = analogRead(TEMP_SENSOR_PIN);

  if((millis()-lastTempPrint) > TEMP_PRINT_INTERVAL)
  {
    //print to LCD screen
    printLCDtemp(currentTemp);
    
    if(currentTemp < 95.0) {
      brew_status_num = 0; //(0) warming up
    }
    else if(currentTemp >= 95.0 && currentTemp < 100.0) {
      brew_status_num = 1; //(1) ready to brew
    }
    else {
      brew_status_num = 2; //(2) Hot
    }
    printBrewstatus(brew_status_num);
    
    //print to serial terminal
    //Serial.print(volt);
    Serial.print("Time: ");
    Serial.print(millis()/1000.0);
    Serial.print(" Temp: ");
    Serial.print(currentTemp);
    //Print a carriage return
    Serial.println();
    
    //increment
    lastTempPrint = millis();
  }

  //PID stuff*************************
  Input = currentTemp;
  myPID.Compute();

  //turn the output pin on/off based on pid output
  //now is current time, windowStartTime was when we started this window + windows size
  unsigned long now = millis();
  if(now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(Output > now - windowStartTime) digitalWrite(RelayPin,HIGH);
  else digitalWrite(RelayPin,LOW);
  //Output is usually the power output of the heater if there was power. With SSR, Output is time on in this window.
  //End PID stuff*********************  
  
  //delay(1000);
}

// read the analog input, convert to degrees C
float getTemperature() { 
  float analogIn = analogRead(TEMP_SENSOR_PIN);
  //Serial.print(analogIn);
  // divide by 1023, the maximum possible input value, that scales the input between 0 - 1
  // then multiply by the reference voltage, which scales 0-1 to 0-vREF (default is 5V)
  // lastly, multiply by 100 to scale it to 10s of millivolts or degrees
  
  return analogIn / 1024.0 * ANALOG_VOLTAGE_REFERENCE * 100.0;
}

void printLCDtemp(float temp)
{
  mySerial.write(254); // cursor to 11th position on second line
  mySerial.write(202);
  
  mySerial.print(temp);
  mySerial.write(254); //cursor to last character on second line
  mySerial.write(207);
  mySerial.print((char)223); //degree sign
  
  return;
}

void printBrewstatus(int status_num)
{
  if(status_num == 0) {
    mySerial.write(254); // cursor to 9th position on second line
    mySerial.write(192);
    mySerial.print("Heating");
  }
  else if(status_num == 1) {
    mySerial.write(254); // cursor to 9th position on second line
    mySerial.write(192);
    mySerial.print("Ready  ");
  }
  else if(status_num == 2) {
    mySerial.write(254); // cursor to 9th position on second line
    mySerial.write(192);
    mySerial.print("Hot    ");
  }
  
  return;
}



