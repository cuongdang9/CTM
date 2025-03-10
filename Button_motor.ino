#define NutGate 34      //Dong/mo cua cong
#define NutLed  35  
#define Led1  32 

#include <Stepper.h>
#define IN1 13
#define IN2 12
#define IN3 14
#define IN4 27
const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

int TTGate = 0;
int TTLed = 0;

void ButtonGate();
void ButtonLed();

void setup() 
{
  Serial.begin(115200);
  pinMode(NutGate, INPUT_PULLUP);
  pinMode(NutLed, INPUT_PULLUP);
  pinMode(Led1, OUTPUT);

  digitalWrite(Led1, LOW);
  
  myStepper.setSpeed(15);   //max speed = 15
}

void loop() 
{
  ButtonGate();
   ButtonLed();
}

void ButtonGate()
{
  if( digitalRead(NutGate) == 0 )
  {
    delay(200);
    while(1)
    {
      //Tha tay ra
      if( digitalRead(NutGate) == 1 )
      {
        if(TTGate == 0)
        {
          Serial.println("Open");
          myStepper.step(stepsPerRevolution);
          delay(1000);
          TTGate = 1;
        }
        else if(TTGate == 1)
        {
          Serial.println("Close");
          myStepper.step(-stepsPerRevolution);
          delay(1000);
          TTGate = 0;
        }
        delay(300);
        break;
        
      }
    }
  }
  
}

void ButtonLed()
{
  if( digitalRead(NutLed) == 0 )
  {
    delay(300);
    while(1)
    {
      //Tha tay ra
      if(digitalRead(NutLed) == 1)
      {
        //thuc hien lenh
        if(TTLed == 0)
        {
          Serial.println("Bat den");
          digitalWrite(Led1,LOW);
          TTLed = 1;
        }
        else if(TTLed == 1)
        {
          Serial.println("Tat den");
          digitalWrite(Led1,HIGH);
          TTLed = 0;
        }
        delay(300);
        break;
      }
    } 
  }  
}
