//#include <ros.h>
//#include <std_msgs/String.h>
#include <Arduino.h>
#include <Bounce2.h>

// project made using an arduino nano, rgb led and 2 buttons
// will be a hardware interface for an estop

// colour pins for led
#define redPin 11
#define greenPin 10
#define bluePin 9

// button pins
#define redButton 3   // eStop
#define blankButton 4 // resume

// creates button objects for debouncer
Bounce2::Button eStop = Bounce2::Button();
Bounce2::Button resume = Bounce2::Button();


const int interval = 50; // debounce interval
int progState = 1;       // state machine position
int stopState = 0;       // estop latch, so that non latching button works
int toggle = 1;          // toggle for led test

// function declarations
void setColor(int redValue, int greenValue, int blueValue); // allows you to set colour by rgb values
void setGreen();                                            // sets green colour
void setColour(String colour);                              // set colour by name (red, orange, green)
void input();                                               // type colour into serial monitor (untested)
void ledTest(int toggle);                                   // switch case funtion that runs various led tests based on toggle
void buttonTest();                                          // test estop button state
void stateMachine(int state, int stp);                      // state machine that has 3 states, running, stopped and armed



void setup()
{

  Serial.begin(9600);
  Serial.println("running");

// led init
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
// button init

  eStop.attach(redButton, INPUT_PULLUP);
  eStop.interval(interval);
  eStop.setPressedState(LOW);

  resume.attach(blankButton, INPUT_PULLUP);
  resume.interval(interval);
  resume.setPressedState(LOW);
 



}

void loop()
{
 
  stateMachine(progState, stopState);
 
  
}

void setColor(int redValue, int greenValue, int blueValue)
{
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}

void stateMachine(int state, int stp)
{

  eStop.update();
  resume.update();

  //Serial.println(eStop.pressed());
  switch (state)
  {
  case 1: //running
    //Serial.println("running");
    setColour("green");
    if (eStop.pressed())
    {
      Serial.println("stopped");
      progState = 2;
      stopState = 1; 
    }
    
    break;
  case 2: //stopped
    
    setColour("red");
    if (eStop.pressed()){
      stopState = 0;
    }
    if (stp == 0)
    {
      progState = 3;
    }
    
    break;
  case 3: // armed
    //Serial.println("stopped"); // can be changed to armed if you want it to publish armed state
    setColour("orange");
    if (eStop.pressed())
    {
      stopState = 1;
    }
    if (stp == 1){
      progState = 2;
    }
    if ((stp == 0) && (resume.pressed()))
    {
      Serial.println("running"); 
      progState = 1;
    }
    break;
  default:
    break;
  }

}

void buttonTest(){
  eStop.update();
  resume.update();


  if (eStop.pressed())
  {
    stopState = !stopState;
  }
  
  Serial.println(stopState);
  
  delay(500);
}
void setGreen()
{
  analogWrite(redPin, 0);
  analogWrite(greenPin, 255);
  analogWrite(bluePin, 0);
}

void setColour(String colour)
{

  if (colour == "green")
  {
    //Serial.println("green");
    analogWrite(redPin, 0);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 0);
  }
  if (colour == "red")
  {
   // Serial.println("red");
    analogWrite(redPin, 255);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 0);
  }
  if (colour == "orange")
  {
   // Serial.println("orange");
    analogWrite(redPin, 255);
    analogWrite(greenPin, 60);
    analogWrite(bluePin, 0);
  }
  
}

void input()

{
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    setColour(input);
  }
}

void ledTest(int toggle)
{
  switch (toggle)
  {
  case 1:
    setColour("green");
    delay(1000);
    setColour("orange");
    delay(1000);
    setColour("red");
    delay(1000);
    break;
  case 2:
    input();
  case 3:
    setColor(255, 0, 0); // Red Color
    delay(1000);
    setColor(0, 255, 0); // Green Color
    delay(1000);
    setColor(0, 0, 255); // Blue Color
    delay(1000);
    setColor(255, 255, 255); // White Color
    delay(1000);
    setColor(170, 0, 255); // Purple Color
    delay(1000);
  case 4:
    setColour("red");
    delay(1000);
  case 5:
    setColor(255, 0, 0); // Red Color
    delay(1000);
  default:
    break;
  }
}