#include <HardwareSerial.h>
#include <ArduinoJson.h>
#define RXD2 16
#define TXD2 17
String inputString = "";
bool stringComplete = false;
#define mySerial Serial2
void Read_ESP32CAM(void);

//===================================================


#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define  CLK_PIN  18 // or SCK
#define DATA_PIN  23 // or MOSI
#define CS_PIN    5 // or SS
MD_Parola myDisplay = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

uint8_t scrollSpeed = 70;    // Gia tri cang thap thi scrolling text cang nhanh
textEffect_t scrollEffect = PA_SCROLL_LEFT;
textPosition_t scrollAlign = PA_LEFT;
uint16_t scrollPause = 1000; // Do tre cua text

#define    BUF_SIZE    75
char curMessage[BUF_SIZE] = { "" };
//char newMessage[BUF_SIZE] = { "" };
bool newMessageAvailable = true;

/* ULN2003      ESP32
  -----------------------
    IN1           D13
    In2           D12
    IN3           D14
    IN4           D27
  -------------------------
*/
#include <Stepper.h>
#define IN1 13
#define IN2 12
#define IN3 14
#define IN4 27
const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

#define NutGate 34      //Dong/mo cua cong
#define NutLed  35      //Bat/tat den
#define NutBuz 32
#define Led1  33
#define Led2  25
#define Buzzer  26

int TTLed = 0;
int TTGate = 0;
int TTBuzz = 0;
String message = "";
long last = 0;

String ChuoiSendEsp32Cam = "";

void StartMatrix(void);
void Matrix(void);

unsigned long Last3 = 0;

unsigned long LastBuzz = 0;
void setup()
{

  Serial.begin(115200);
  while (!Serial);
  mySerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  while (!mySerial);

  pinMode(NutGate, INPUT_PULLUP);
  pinMode(NutLed, INPUT_PULLUP);
  pinMode(NutBuz, INPUT_PULLUP);
  pinMode(Led1, OUTPUT);
  pinMode(Led2, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  digitalWrite(Led1, LOW);
  digitalWrite(Led2, LOW);
  digitalWrite(Buzzer, LOW);
  StartMatrix();
  last = millis();
  Serial.println("Start");
  Last3 = millis();
  LastBuzz = millis();

}

void loop()
{

  Matrix();
  ButtonGate();
  ButtonLed();
  ButtonBuzzer();
  Read_ESP32CAM();
  StopBuzz();

  /*
    if(millis() - Last3 >= 1000 )
    {
    Serial.print("digitalRead(NutLed):");
    Serial.println(digitalRead(NutLed));
    Serial.print("digitalRead(NutGate):");
    Serial.println(digitalRead(NutGate));

     Last3 = millis();
    }

  */

}
void StopBuzz(void)
{
  if (TTBuzz == 1)
  {
    if ( millis() - LastBuzz >= 1000)
    {
      TTBuzz = 0;
      digitalWrite(Buzzer, LOW);
      LastBuzz = millis();
    }
  }

}
void Matrix(void)
{
  if (myDisplay.displayAnimate())
  {
    myDisplay.displayReset();
  }
}
void Read_ESP32CAM(void)
{
  while (mySerial.available())
  {
    const size_t capacity = JSON_OBJECT_SIZE(2) + 256;
    DynamicJsonDocument JSON(capacity);
    DeserializationError error = deserializeJson(JSON, mySerial);
    if (error)
    {
      return;
    }
    else
    {
      Serial.println();
      Serial.println("Data nhận được ESP32 - CAM là:");
      serializeJsonPretty(JSON, Serial);
      Serial.println();
      /*
        ESP32 CAM GỬI {"SEND":"1"} CHO ESP32 => DataJson(String (TTGate), String (TTLed), String(message));
        CAM {"MESS":"ABC"} => ESP32 LAY ABC => HIEN THI MATRIX
        {"TTLED":"1"} => LED OFF
        {"TTLED":"0"} => LED ON
        {"TTGATE":"0"} => CUA OPEN
        {"RIFD":"0"} => OPEN
        {"RIFD":"1"} => CLOSE
      */
      if (JSON.containsKey("SEND"))
      {
        Serial.println("SEND ESP32 CAM");
        DataJson(String (TTGate), String (TTLed), String(message));
      }
      else if (JSON.containsKey("RFID"))
      {
        // điều khiển motor bằng rfid
        Serial.println("Điều khiển motor bằng RFID");
        if (TTGate == 1)
        {
          Serial.println("CLOSE");
          myStepper.step(2*stepsPerRevolution);
          TTGate = 0;
          StartMatrix();
        }
        else if (TTGate == 0)
        {
          Serial.println("OPEN");
          myStepper.step(2*-stepsPerRevolution);
          TTGate = 1;
          StartMatrix();
        }
        Serial.println("SEND ESP32 CAM");
        DataJson(String (TTGate), String (TTLed), String(message));
      }
      else if (JSON.containsKey("MESS"))
      {
        // Mess matrix
        //{"MESS":"ABC"}
        Serial.println("Mess matrix");
        String Data_MESS = JSON["MESS"];
        Serial.print("Data_MESS:");
        Serial.println(Data_MESS);

        char data[50];
        //String message;
        message = (String)Data_MESS;
        message.toCharArray(data, 50);
        strcpy(curMessage, data);

        StartMatrix();
        Serial.println("SEND ESP32 CAM");
        DataJson(String (TTGate), String (TTLed), String(message));
      }
      else if (JSON.containsKey("TTLED"))
      {
        // điều khiển led
        String Data_TTLED = JSON["TTLED"];
        Serial.print("Data_TTLED:");
        Serial.println(Data_TTLED);
        if (Data_TTLED.toInt() == 1)
        {
          Serial.println("Bat den");
          digitalWrite(Led1, LOW);
          digitalWrite(Led2, LOW);
          TTLed = 0;

        }
        else if (Data_TTLED.toInt() == 0)
        {
          Serial.println("Tat den");
          digitalWrite(Led1, HIGH);
          digitalWrite(Led2, HIGH);
          TTLed = 1;
        }
        Serial.println("SEND ESP32 CAM");
        DataJson(String (TTGate), String (TTLed), String(message));
      }
      else if (JSON.containsKey("TTGATE"))
      {
        // điều khiển cửa

        String Data_TTGATE = JSON["TTGATE"];
        Serial.print("Data_TTGATE:");
        Serial.println(Data_TTGATE);
        if (Data_TTGATE.toInt() == 1)
        {
          myStepper.step(2*stepsPerRevolution);
          //(1000);
          Serial.println("Openning the gate!!!");
          TTGate = 0;
        }
        else if (Data_TTGATE.toInt() == 0)
        {
          myStepper.step(2*-stepsPerRevolution);
          //delay(1000);
          Serial.println("Closing the gate!!!");
          TTGate = 1;
        }
        Serial.println("SEND ESP32 CAM");
        DataJson(String (TTGate), String (TTLed), String(message));
      }
      JSON.clear();
    }
  }
}
void ButtonGate()
{
  if ( digitalRead(NutGate) == 0 )
  {
    delay(200);
    while (1)
    {
      //Tha tay ra
      Matrix();
      if ( digitalRead(NutGate) == 1 )
      {
        if (TTGate == 1)
        {
          Serial.println("CLOSE");
          myStepper.step(2*stepsPerRevolution);

          //delay(1000);
          TTGate = 0;
          StartMatrix();
        }
        else if (TTGate == 0)
        {
          Serial.println("OPEN");
          myStepper.step(2*-stepsPerRevolution);
          //delay(1000);
          StartMatrix();
          TTGate = 1;

        }
        delay(300);
        break;

      }
    }
  }

}

void ButtonLed()
{
  if ( digitalRead(NutLed) == 0 )
  {
    delay(300);
    while (1)
    {
      //Tha tay ra
      Matrix();
      if (digitalRead(NutLed) == 1)
      {
        //thuc hien lenh
        if (TTLed == 1)
        {
          Serial.println("Bat den");
          digitalWrite(Led1, LOW);
          digitalWrite(Led2, LOW);
          TTLed = 0;
          StartMatrix();
        }
        else if (TTLed == 0)
        {
          Serial.println("Tat den");
          digitalWrite(Led1, HIGH);
          digitalWrite(Led2, HIGH);
          TTLed = 1;
          StartMatrix();
        }
        delay(300);
        break;
      }
    }
  }
}
void ButtonBuzzer(void)
{
  if ( digitalRead(NutBuz) == 0 )
  {
    Serial.println("nut buzz start!");
    LastBuzz = millis();
    delay(300);
    while (1)
    {
      //Tha tay ra
      Matrix();
      LastBuzz = millis();
      if (digitalRead(NutBuz) == 1)
      {
        digitalWrite(Buzzer, HIGH);
        LastBuzz = millis();
        TTBuzz = 1;
        StartMatrix();
        delay(300);
        break;
      }
    }
  }
}

void DataJson(String TTGate, String TTLed, String Send)
{
  ChuoiSendEsp32Cam = "{\"TTGATE\":\"" + String(TTGate) + "\"," +
                      "\"TTLED\":\"" + String(TTLed) + "\"," +
                      "\"TTSEND\":\"" + String(Send)      + "\"}";

  Serial.print("ChuoiSendEsp32Cam:");
  Serial.println(ChuoiSendEsp32Cam);

  mySerial.println(ChuoiSendEsp32Cam);
  mySerial.flush();
}
void StartMatrix(void)
{
  myDisplay.begin();
  myDisplay.setIntensity(0);    // Set the intensity (brightness) of the display (0-15)
  myDisplay.displayText(curMessage, scrollAlign, scrollSpeed, scrollPause, scrollEffect, scrollEffect);
  myStepper.setSpeed(15);
}
