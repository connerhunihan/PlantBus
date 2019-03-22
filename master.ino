/* 
  Plant Bus master file
  Critical Making, 2019
*/
 
// GENERAL SETUP
  int counter;
  int maxmileage = 20; // update to whatever you think the daily max walking distance will be
  int milestoneSize = 5;
  int milestone1 = (maxmileage / 5) * 1;
  int milestone2 = (maxmileage / 5) * 2;
  int milestone3 = (maxmileage / 5) * 3;
  int milestone4 = (maxmileage / 5) * 4;
  int milestone5 = (maxmileage / 5) * 5;
  bool milestoneRan = false;



// BREAKBEAM SETUP
  #define SENSORPIN 13 // Pin 13 for break beam sensor 
  int sensorState = 0, lastState=0;         // variable for reading the breakbeam status



// CLOCK SETUP
  #include <Adafruit_GFX.h>
  #include "Adafruit_LEDBackpack.h"
  #if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
    // Required for Serial on Zero based boards
    #define Serial SERIAL_PORT_USBVIRTUAL
  #endif
  Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();
  char displaybuffer[4] = {' ', ' ', ' ', ' '};
  char s[4];    // takes the int count and splits it into 4 digits 


// LED SETUP
  int led1 = 5; 
  int led2 = 6;
  int led3 = 9;
  int led4 = 10;
  int led5 = 19;


// PIEZO SETUP  
  #include "pitches.h"
  #include "pitchesJessy.h"
  int speakerPin = 15;
  int toneVal;
  
  int canonMelody[] = {
    NOTE_G4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_G3, NOTE_A3, NOTE_B3, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4
  };
  int jasmineMelogy[] = {
    NOTE_E4, NOTE_D4, NOTE_E4, NOTE_G4, NOTE_A4, NOTE_G4, NOTE_C5, NOTE_A4, NOTE_G4, NOTE_E4, NOTE_G4, NOTE_A4,
    NOTE_C5, NOTE_D5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_C5, NOTE_C5, NOTE_G4
  };
  int eliseMelody[] = {
    NOTE_E4, NOTE_DS4, NOTE_E4, NOTE_DS4, NOTE_E4, NOTE_B3, NOTE_D4, NOTE_C4, NOTE_A3
  };
  int canonNoteDurations[] = {
    2, 4, 4, 2, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4
  };
  int jasmineNoteDurations[] = {
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1, 2,
    2, 4, 4, 4, 4, 4, 4, 2, 2
  };
  int eliseNoteDurations[] {
    4, 4, 4, 4, 4, 4, 4, 4, 2
  };
  int noteDuration = 10; // ms

  
// SERVO SETUP
  // for color reference https://www.mysensors.org/build/servo
  #include <Servo.h> 
  int SERVOPIN = 11;
  String readString; //String captured from serial port
  Servo myservo;  // create servo object to control a servo 
  int n; //value to write to servo


// RELAY SETUP
  int relayPin = 12;// set pin 12 for relay output


// FEATHER SETUP
  #include <Arduino.h>
  #include <SPI.h>
  #include "Adafruit_BLE.h"
  #include "Adafruit_BluefruitLE_SPI.h"
  #include "Adafruit_BluefruitLE_UART.h"
  
  #include "BluefruitConfig.h"
  
  #if SOFTWARE_SERIAL_AVAILABLE
    #include <SoftwareSerial.h>
  #endif
  
  /* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
  Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
  

  // A small helper
  void error(const __FlashStringHelper*err) {
    Serial.println(err);
    while (1);
  }
  
  /* The service information */
  
  int32_t hrmServiceId;
  int32_t hrmMeasureCharId;
  int32_t hrmLocationCharId;
  
  
void setup() {
// GENERAL SETUP
  Serial.begin(9600);
  counter = 0;

// BREAK BEAM SETUP 
  pinMode(SENSORPIN, INPUT);  // initialize the sensor pin as an input:   
  digitalWrite(SENSORPIN, HIGH); // turn on the pullup

// LED CLOCK  
  s[0] = ' '; // setting initial screen to blank
  s[1] = ' ';
  s[2] = ' ';
  s[3] = ' ';
  
  alpha4.begin(0x70);  // pass in the address

  alpha4.writeDigitRaw(3, 0x0);
  alpha4.writeDigitRaw(0, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(0, 0x0);
  alpha4.writeDigitRaw(1, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(1, 0x0);
  alpha4.writeDigitRaw(2, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(2, 0x0);
  alpha4.writeDigitRaw(3, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  
  alpha4.clear();
  alpha4.writeDisplay();

// PIEZO SETUP
//  pinMode(piezo, OUTPUT);
  pinMode(speakerPin, OUTPUT);
//  a = 1;
//  b = 0;

// LED SETUP
  pinMode(led1, OUTPUT); // Declare the LED as an output
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(led5, OUTPUT);

// SERVO SETUP
  myservo.writeMicroseconds(1500); //set initial servo position if desired
  myservo.attach(SERVOPIN, 500, 2500);  //the pin for the servo control, and range if desired
  
// RELAY SETUP
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); // set relay to be off at setup

// FEATHER SETUP
Serial.begin(9600);

if ( !ble.begin(VERBOSE_MODE) )
{
  error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
}
Serial.println( F("OK!") );

/* Perform a factory reset to make sure everything is in a known state */
Serial.println(F("Performing a factory reset: "));
if (! ble.factoryReset() ){
     error(F("Couldn't factory reset"));
}

/* Disable command echo from Bluefruit */
ble.echo(false);

Serial.println("Requesting Bluefruit info:");
/* Print Bluefruit information */
ble.info();

/* Change the device name to make it easier to find */
Serial.println(F("Setting device name to 'Conner's Fearther': "));

if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Conner's Feather`")) ) {
  error(F("Could not set device name?"));
}

/* Add the Heart Rate Service definition */
/* Service ID should be 1 */
Serial.println(F("Adding the Heart Rate Service definition (UUID = 0x180D): "));
 ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x180D"), &hrmServiceId);

/* Add the Heart Rate Measurement characteristic */
/* Chars ID for Measurement should be 1 */
Serial.println(F("Adding the Heart Rate Measurement characteristic (UUID = 0x2A37): "));
ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &hrmMeasureCharId);

/* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
Serial.print(F("Adding Heart Rate Service UUID to the advertising payload: "));
ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );

/* Reset the device for the new service setting changes to take effect */
Serial.print(F("Performing a SW reset (service changes require a reset): "));
ble.reset();

Serial.println();

}

void loop() {
// set servo to starting position at counter = 0
if (counter == 0) {
    myservo.write(180);
  }
  
// BREAKBEAM  
  // read the state of the pushbutton value:
  sensorState = digitalRead(SENSORPIN);
 
  // check if the sensor beam is broken
  // if it is, the sensorState is LOW:
  
  if (sensorState && !lastState) {
    Serial.println("Unbroken");
  } 
  if (!sensorState && lastState) {
    Serial.println("Broken");
    counter += 1;
    Serial.println(counter);
  }
  //delay(10);
  lastState = sensorState;

  // FEATHER
  featherRun();

  // CLOCK
  LEDclock();
  
  // SERVO 
  servoMove();

  
  // If milestone reached, execute
  // if ((counter != 0) && (counter % milestoneSize == 0)) {
  if ((counter == milestone1) ||
      (counter == milestone2) ||
      (counter == milestone3) ||
      (counter == milestone4) ||
      (counter == milestone5)) {
  
     LED();
     relay();
     piezoPlay();
     counter += 1;
     //milestoneRan = true;
  } //else {
     //milestoneRan = false;
  //}

}


void LEDclock() {  
  sprintf(s,"%ld", counter);  // 

    // scroll down display
    displaybuffer[0] = s[0];
    displaybuffer[1] = s[1];
    displaybuffer[2] = s[2];
    displaybuffer[3] = s[3];
   
    // set every digit to the buffer
    alpha4.writeDigitAscii(0, displaybuffer[0]);
    alpha4.writeDigitAscii(1, displaybuffer[1]);
    alpha4.writeDigitAscii(2, displaybuffer[2]);
    alpha4.writeDigitAscii(3, displaybuffer[3]);
    // write it out!
    alpha4.writeDisplay();
    delay(10);

    Serial.print("Wheel counter: ");
    Serial.println(counter);
    
}

void LED() {
  if (counter == milestone1) {  // Turn LEDs on at milestones
    digitalWrite(led1, HIGH); 
  }
  else if (counter == milestone2) {
    digitalWrite(led2, HIGH); 
  }
  else if (counter == milestone3) {
    digitalWrite(led3, HIGH); 
  }
  else if (counter == milestone4) {
    digitalWrite(led4, HIGH); 
  }
  else if (counter == milestone5) {
    digitalWrite(led5, HIGH); 
  }
  
//  [CH] took this out because we want all the lights to stay on when the plant has had enough water
//  else if (counter > milestone5) {
//    digitalWrite(led1, LOW); // Turn the LED off at max mileage
//    digitalWrite(led2, LOW);
//    digitalWrite(led3, LOW);
//    digitalWrite(led4, LOW);
//    digitalWrite(led5, LOW);
//  }
} 

void servoMove() {
  int angle;
  if (counter == 0) {
    angle = 180;
  }
  else if (counter <= milestone1 ) {  
    angle = map(counter, 0, milestone1, 180, 0);
    myservo.write(angle);
    
  }
  else if (counter > milestone1 && counter <= milestone2) {
    angle = map(counter, milestone1, milestone2, 180, 0);
    myservo.write(angle);
  }
  else if (counter > milestone2 && counter <= milestone3) {
    angle = map(counter, milestone2, milestone3, 180, 0);
    myservo.write(angle);
  }
  else if (counter > milestone3 && counter < milestone4) {
    angle = map(counter, milestone3, milestone4, 180, 0);
    myservo.write(angle);
  }
  else if (counter > milestone4 && counter < milestone5) {
    angle = map(counter, milestone4, milestone5, 180, 0);
    myservo.write(angle);
  }
  else if (counter > milestone5) {
    angle = 0; 
    myservo.write(angle);
  }
}

void relay() {
    //piezoPlay();
    digitalWrite(relayPin, HIGH);
    delay(2000);
    digitalWrite(relayPin, LOW);
}

void piezoPlay() {
    for (int thisNote = 0; thisNote < 9; thisNote++) {
      int noteDuration = 1000 / eliseNoteDurations[thisNote];
      tone(speakerPin, eliseMelody[thisNote], noteDuration);
      
      int pauseBetweenNotes = noteDuration;
      delay(pauseBetweenNotes);
      
      noTone(speakerPin);
    }
}


void featherRun() {
  /* Command is sent when \n (\r) or println is called */
  /* AT+GATTCHAR=CharacteristicID,value */
  ble.print( F("AT+GATTCHAR=") );
  ble.print( hrmMeasureCharId );
  ble.print( F(",00-") );
  ble.println(counter, HEX);

  /* Check if command executed OK */
  if ( !ble.waitForOK() )
  {
    Serial.println(F("Failed to get response!"));
  }

  /* Delay before next measurement update */
  delay(1000);
}
