#include <Arduino.h>
#include <Bounce2.h>

// project made using an arduino nano, rgb led and 2 buttons
// will be a hardware interface for an estop

// colour pins for led
#define redPin 11
#define greenPin 10
#define bluePin 9

// button pins
#define redButton 3
#define blankButton 4

//Bounce eStop = Bounce();
//Bounce resume = Bounce();

Bounce2::Button eStop = Bounce2::Button();
Bounce2::Button resume = Bounce2::Button();

const int interval = 50;
int progState = 1;
int stopState = 0;


void setColor(int redValue, int greenValue, int blueValue);
void setGreen();
void setColour(String colour);
void input();
void ledTest(int toggle);
void buttonTest();
void stateMachine(int state, int stp);

    int toggle = 1; // toggle for led test

void setup()
{

  Serial.begin(9600);
  Serial.println("serial started");

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
 // ledTest(toggle);
  stateMachine(progState, stopState);
 //buttonTest();
  
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
    Serial.println("running");
    setColour("green");
    if (eStop.pressed())
    {
      Serial.println("stopped");
      progState = 2;
      stopState = 1;
      
    }
    
    break;
  case 2: //stopped
   // Serial.println("stopped");
    setColour("red");
    Serial.println(stopState);
    if (eStop.pressed()){
      stopState = 0;
    }
    if (stp == 0)
    {
      progState = 3;
    }
    
    break;
  case 3: // armed
    Serial.println("armed");
    setColour("orange");
    if ((stp == 0) && (resume.pressed()))
    {
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