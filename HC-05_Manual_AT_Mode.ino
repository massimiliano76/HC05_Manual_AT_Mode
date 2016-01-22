/*
 * HC-05 Manual AT Mode
 * This sketch let's you communicate with an HC-05 bluetooth module,
 * sending AT commands and receiving responses.
 * 
 * This sketch will take care of putting the HC-05 in AT Mode,
 * however most HC-05 modules do not have a pin soldered for the KEY pin.
 * The KEY pin is pin 34 on the module (in the corner of the module),
 * it is necessary to solder a wire to pin 34 so that the Arduino or compatible MCU
 * will be able to control the KEY pin. 
 * When the KEY pin is held HIGH while powering the module,
 * the module enters AT Mode (LED on the module flashes every two seconds). 
 * Even in Communication Mode (KEY pulled LOW or left floating), while the module is not actually connected to a device
 * the module will respond to a limited number of AT Commands.
 * In order to turn the module on and off, we use the module's ENABLE pin,
 * which is more of a "DISABLE" pin, because it will disable the module when pulled low.
 * This allows us to switch between AT Mode and Communication Mode by disabling the module and pulling the KEY pin HIGH or LOW accordingly, 
 * then pulling the ENABLE pin HIGH again.
 * 
 * The baud rate depends on the module's firmware, in this case it is 38400.
 * The Arduino Serial Monitor must be set to send "CR+LF".
 * 
 * The LED is optional but will use the reading on the HC-05's STATE pin
 * to detect when the module is connected to and communicating with a device.
 * 
 * This sketch is made for an Atmega 1284P on a breadboard, using the "Maniacbug Mighty 1284P 16MHZ using Optiboot" bootloader (see JChristensen's fork).
 * The Atmega 1284P has more than one hardware UART, so we are able to use Serial and Serial1
 * rather than SoftwareSerial. For other boards that have only one UART, a SoftwareSerial may be implemented instead.
 * However I personally have noticed that SoftwareSerial does not always work very well,
 * often showing scrambled characters, so I prefer using a board that has more than one hardware UART.
 * 
 * We also implement the Timer library here, to avoid using delays in the script.
 * Delays will often be the cause of corrupted or scrambled or spurious serial data.
 * 
 * Wire connections are as follows (make sure the HC-05's pins I/O or serial pins are ever only connected to 3.3V to avoid any damage):
 * (the module I am using is an FC-114, you can refer to this image for the pinout http://www.martyncurrey.com/wp-content/uploads/2015/08/HC-05_FC-114__HC-06_FC-114_001_1600.jpg)
 * (for 1284P pinout see https://maniacbug.wordpress.com/2011/11/27/arduino-on-atmega1284p-4/)
*                          ____
 *    pin 34--KEY       D13 ---|
 *   /   ____                  |
 *  /   |--- STATE      D12 ---|
 * |____|--- RXD    TX1/D17 ---|_____
 * HC-05|--- TXD    RX1/D16 ---|1284P
 * _____|--- GND        GND ---|_____
 *      |--- VCC        VCC ---|
 *      |--- EN         D14 ---|
 *      |____              ____|
 *      
 *      If using an Arduino Mega, Serial1 uses pins 18->TX1 and 19->RX1
 *      
 * Author: John Romano D'Orazio http://www.johnromanodorazio.com
 * Email: priest@johnromanodorazio.com
 * Last modified January 2016
*/

#include "Timer.h"                     //http://github.com/JChristensen/Timer

#define LED             0 // led on digital pin 0

#define HC05_STATE_PIN  12 //hc-05 state pin to 1284P digital pin 12
#define HC05_KEY_PIN    13 //hc-05 key pin (wire soldered to pin 34!) to 1284P digital pin 13
#define HC05_EN_PIN     14 //hc-05 enable pin to 1284P digital pin 14

#define AT_MODE               HIGH
#define COMMUNICATION_MODE    LOW


//instantiate the timer object
Timer t;

int dynamicEvent;

boolean SETTINGHC05MODE = false;
int HC05_MODE;
int currentFunctionStep = 0;

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(HC05_STATE_PIN, INPUT);
  pinMode(HC05_KEY_PIN, OUTPUT);
  pinMode(HC05_EN_PIN, OUTPUT);

  // put your setup code here, to run once:
  Serial.begin(38400);
  while(!Serial){} //wait until Serial is ready
  Serial.println("Serial communication is ready!");
  
  Serial1.begin(38400);
  while(!Serial1){} //wait until Serial1 is ready
  Serial.println("HC-05 serial is ready too!");

  //Start the HC-05 module in AT Mode
  HC05_MODE = AT_MODE;
  SETTINGHC05MODE = true; //PROGRAM STATE FLAG
  Set_HC05_MODE();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  t.update();

  if (Serial.available() > 0) {
    byte outgoing = Serial.read();
    Serial.write(outgoing);
    Serial1.write(outgoing); 
  }

  if (Serial1.available() > 0) {
    if(SETTINGHC05MODE){
      //we'll just throw away any serial data spewing forth while disabling and re-enabling the module
      //spurious serial data is common in such conditions
      Serial1.read(); 
    }
    else{
      Serial.write(Serial1.read()); 
    }
  }

}


void Set_HC05_MODE(){
  if(currentFunctionStep==0){
    Serial.print("Now setting HC-05 mode to ");
    if(HC05_MODE == COMMUNICATION_MODE){
      Serial.println("COMMUNICATION_MODE");  
    }else if(HC05_MODE == AT_MODE){
      Serial.println("AT_MODE");
    }
    digitalWrite(HC05_EN_PIN, LOW); //EN to LOW = disable (pull low to reset when changing modes!)
    currentFunctionStep++;
    dynamicEvent = t.after(200,Set_HC05_MODE);
  }
  else if(currentFunctionStep==1){
    digitalWrite(HC05_KEY_PIN, HC05_MODE); //KEY to HIGH = full AT mode, to LOW = communication mode
    currentFunctionStep++;
    dynamicEvent = t.after(200,Set_HC05_MODE);
  }
  else if(currentFunctionStep==2){
    digitalWrite(HC05_EN_PIN, HIGH); //EN to HIGH = enable
    currentFunctionStep++;
    if(HC05_MODE == AT_MODE){
      dynamicEvent = t.after(3000,Set_HC05_MODE);
    }
    else if(HC05_MODE == COMMUNICATION_MODE){
      dynamicEvent = t.after(5000,Set_HC05_MODE);
    }
  }
  else if(currentFunctionStep==3){
    currentFunctionStep=0;
    SETTINGHC05MODE = false;
    Serial.println(">>READY");
  }  
}

