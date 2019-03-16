/* 
  Plant Bus master file
  Critical Making, 2019
*/
 
// GENERAL SETUP
  int counter;
  int maxmileage = 20; // update to whatever you think the daily max walking distance will be
  int milestone1 = (maxmileage / 4) * 1;
  int milestone2 = (maxmileage / 4) * 2;
  int milestone3 = (maxmileage / 4) * 3;
  int milestone4 = (maxmileage / 4) * 4;

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

// PIEZO SETUP  
  #include "pitches.h"
  int piezo = 15;

  volatile int beatlength = 105 ; // determines tempo
  float beatseparationconstant = 0.3;
  int a; // part index
  int b; // song index
  // Chorus
  int song1_chorus_melody[] =
  { b4f, b4f, a4f, a4f,
    f5, f5, e5f, b4f, b4f, a4f, a4f, e5f, e5f, c5s, c5, b4f,
    c5s, c5s, c5s, c5s,
    c5s, e5f, c5, b4f, a4f, a4f, a4f, e5f, c5s,
    b4f, b4f, a4f, a4f,
    f5, f5, e5f, b4f, b4f, a4f, a4f, a5f, c5, c5s, c5, b4f,
    c5s, c5s, c5s, c5s,
    c5s, e5f, c5, b4f, a4f, rest, a4f, e5f, c5s, rest
  };
  int song1_chorus_rhythmn[] =
  { 1, 1, 1, 1,
    3, 3, 6, 1, 1, 1, 1, 3, 3, 3, 1, 2,
    1, 1, 1, 1,
    3, 3, 3, 1, 2, 2, 2, 4, 8,
    1, 1, 1, 1,
    3, 3, 6, 1, 1, 1, 1, 3, 3, 3, 1, 2,
    1, 1, 1, 1,
    3, 3, 3, 1, 2, 2, 2, 4, 8, 4
  };

// SERVO SETUP
  #include <Servo.h> 
  String readString; //String captured from serial port
  Servo myservo;  // create servo object to control a servo 
  int n; //value to write to servo

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
  pinMode(piezo, OUTPUT);
  a = 0;
  b = 0;

// LED SETUP
  pinMode(led1, OUTPUT); // Declare the LED as an output
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);

// SERVO SETUP
  myservo.writeMicroseconds(1500); //set initial servo position if desired
  myservo.attach(12, 500, 2500);  //the pin for the servo control, and range if desired
}
 
void loop() {

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
  delay(500);
  lastState = sensorState;

// CLOCK
  LEDclock();

// LED
  LED();

// PIEZO
//  piezoPlay();

// SERVO 
  servoMove();

}

void LEDclock() {  
  sprintf(s,"%ld", counter);  // 
    
    //if (! isprint(c)) return; // only printable!
    
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
    delay(50);
}

void LED() {
  if (counter == milestone1) {
    digitalWrite(led1, HIGH); // Turn the LED on
  }
  if (counter == milestone2) {
    digitalWrite(led2, HIGH); 
  }
  if (counter == milestone3) {
    digitalWrite(led3, HIGH); 
  }
  if (counter == milestone4) {
    digitalWrite(led4, HIGH); 
  }
  if (counter == maxmileage) {
    digitalWrite(led1, LOW); // Turn the LED off at max mileage
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    digitalWrite(led4, LOW);
  }
} 

void servoMove() {
  int angle;
  if (counter < milestone1 ) {
    angle = 180 / (milestone1 - counter); 
    myservo.write(angle);
  }
  if (counter > milestone1 && counter < milestone2) {
    angle = 180 / (milestone2 - counter); 
    myservo.write(angle);
  }
  if (counter > milestone2 && counter < milestone3) {
    angle = 180 / (milestone3 - counter); 
    myservo.write(angle);
  }
  if (counter > milestone3 && counter < milestone4) {
    angle = 180 / (milestone4 - counter); 
    myservo.write(angle);
  }
}

//void piezoPlay() {
//  int notelength;
//  if (counter == milestone1) { //chorus
//    notelength = beatlength * song1_chorus_rhythmn[b];
//    if (song1_chorus_melody[b] > 0) {
//      tone(piezo, song1_chorus_melody[b], notelength);
//    }
//    b++;
//    if (b >= sizeof(song1_chorus_melody) / sizeof(int)) {
//      a++;
//      b = 0;
//    }
//  }
//  delay(notelength); // necessary because piezo is on independent timer
//  noTone(piezo);
//  delay(notelength * beatseparationconstant); // create separation between notes
//}
