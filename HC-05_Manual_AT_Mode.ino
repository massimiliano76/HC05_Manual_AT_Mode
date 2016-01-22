/*
 * HC-05 Manual AT Mode
 * Author: John Romano D'Orazio http://www.johnromanodorazio.com
 * Email: priest@johnromanodorazio.com
*/

#include "Timer.h"                     //http://github.com/JChristensen/Timer

#define LED             0 // led digital pin

#define HC05_STATE_PIN  12 //hc-05 state digital pin
#define HC05_KEY_PIN    13 //hc-05 key digital pin
#define HC05_EN_PIN     14 //hc-05 enable digital pin

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

  //Start the HC-05 module in at mode
  HC05_MODE = AT_MODE;
  SETTINGHC05MODE = true;
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
      Serial1.read(); //just throw it away, don't do anything about it
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
      //DO_ADCN = true;
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

