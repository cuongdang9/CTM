
#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include "web.h"
#include <ESPmDNS.h>

#include <WebSocketsServer.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

//const char* ssid = "Sang";
//const char* password = "08061993";                                                       
//const char* ssid = "AB";
//const char* password = "1234567890";
const char* ssid = "nguyenhung";
const char* password = "123456789";
bool isClientConnected = false;

WebServer server(80); // chạy web
WebSocketsServer webSocket = WebSocketsServer(81); // show va dieu khien
#include <ArduinoJson.h>
#define RXD2 4
#define TXD2 2
#define mySerial Serial2
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;

#include <SPI.h>
#include <MFRC522.h>
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS 15
#define RST_PIN  16
MFRC522 mfrc522(HSPI_SS, RST_PIN);
String MaThe = "";
//String MaThe1 = "d2 d3 d0 1b";
//String MaThe2 = "a2 83 c5 1b";

//Ma 2 the mau xanh
String MaThe1 = "51 30 cd 3b";  
String MaThe2 = "8c cd e0 37";
void Read_RFID(void);


int TTLed = 0;
int TTGate = 0;
String message = "";
String Client = "ESP";
long last = 0;
int bien = 0;
long last1 = 0;
void CameraInit();
void OpenDns(void);


void SendWebPage()
{
  server.send(200, "text/html", webpage );
}
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

  switch (type) {
    case WStype_DISCONNECTED:
      {
        Serial.printf("[%u] Disconnected!\n", num);
        isClientConnected = false;
      }
      break;
    case WStype_CONNECTED:
      {
        last = millis();
        IPAddress ip = webSocket.remoteIP(num);
        Serial.print("Connect IP address: ");
        Serial.println(ip);
        isClientConnected = true; // khi co client connect => json + ảnh
      }
      break;
    case WStype_TEXT:
      //Serial.printf("Data nhận được: %s\n", payload);
      String Data = "";
      Data = (char*)payload;
      // indexOf kiểm tra tồn tại kí tự gì đó , nếu có tồn tại >= 0
      // gửi qua ESP32
      Serial.print("Data web send esp32:");
      Serial.println(Data);
      mySerial.println(Data);
      mySerial.flush();
      last1 = millis();
      last = millis();


      break;
  }
}

String DataSendESP32 = "";

#include <Ticker.h>
Ticker ticker;
void tick()
{
  Serial.println("tich!!!");
}
void setup()
{
  Serial.begin(115200);
  while (!Serial);
  mySerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  while (!mySerial);
  Serial.setDebugOutput(true);
  Serial.println();

  SPI.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI);

  mfrc522.PCD_Init();

  delay(100);


  CameraInit();

  delay(2000);

  ConnectWiFi();
  delay(2000);

 ticker.detach();

 Serial.println("Stop!!!");

  delay(500);

  OpenDns();

  server.on("/", [] {
    SendWebPage();
  });

  server.begin();

  webSocket.begin();

  webSocket.onEvent(webSocketEvent);
  last = millis();
  last1 = millis();
}
void loop()
{
  duytriServer();

  SendDataCamera();

  SendJson();

  Read_RFID();

}
void Read_RFID(void)
{

  if (mfrc522.PICC_IsNewCardPresent())
  {
    if (mfrc522.PICC_ReadCardSerial())
    {
      MaThe = "";
      for (byte i = 0; i < mfrc522.uid.size; i++)
      {

        MaThe.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
        MaThe.concat(String(mfrc522.uid.uidByte[i], HEX));
      }

      Serial.print("MaThe:");
      Serial.println(MaThe);
      if (MaThe.indexOf(MaThe1) != -1)
      {

        Serial.println("RFID SEND OK");
       mySerial.println("{\"RFID\":\"OK\"}");
       mySerial.flush();
      }
      else if (MaThe.indexOf(MaThe2) != -1)
      {

        Serial.println("RFID SEND OK");
        mySerial.println("{\"RFID\":\"OK\"}");
       mySerial.flush();
      }
      last1 = millis();
      mfrc522.PICC_HaltA();
    }
  }
}

void duytriServer()
{
  Read_ESP32CAM();
  webSocket.loop();
  server.handleClient();
  if (millis() - last1 >= 2000)
  {
    if (isClientConnected)
    {
     
      DataSendESP32 = "";
      DataSendESP32 = "{\"SEND\":\"1\"}";
      Serial.print("DataSendESP32:");
      Serial.println(DataSendESP32);
      mySerial.println(DataSendESP32);
      mySerial.flush();
    }
    last1 = millis();

  }
}

void Read_ESP32CAM()
{
  while (mySerial.available())
  {
    SendDataCamera();
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
      Serial.println("Data nhận được ESP32 là:");
      serializeJsonPretty(JSON, Serial);
      Serial.println();
      if(JSON.containsKey("TTGATE"))
      {
        String Data_TTGATE = JSON["TTGATE"];
        TTGate = Data_TTGATE.toInt();
      }
      if(JSON.containsKey("TTLED"))
      {
        String Data_TTLED = JSON["TTLED"];
        TTLed = Data_TTLED.toInt();
      }
      if(JSON.containsKey("TTSEND"))
      {
        String Data_TTSEND = JSON["TTSEND"];
        message = (String)Data_TTSEND;
      }
      JSON.clear();
    }
  }
}
void OpenDns(void)
{
  if (!MDNS.begin("esp32")) {
    Serial.println("Error starting mDNS");
    return;
  }
  MDNS.addService("http", "tcp", 80);
}
void SendDataCamera()
{
  if (isClientConnected)
  {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    fb = esp_camera_fb_get();
    if (!fb)
    {
      Serial.println("Camera capture failed");
      esp_camera_fb_return(fb);
      return;
    }
    size_t fb_len = 0;
    if (fb->format != PIXFORMAT_JPEG)
    {
      Serial.println("Non-JPEG data not implemented");
      return;
    }
    webSocket.broadcastBIN((const uint8_t*) fb->buf, fb->len);
    esp_camera_fb_return(fb);
  }
}
void SendJson()
{
  if (millis() - last >= 1000)
  {

    if (isClientConnected)
    {
      DataJson(String(Client) ,  String(TTGate),  String(TTLed) , String(message));
    }

    last = millis();
  }
}

void ConnectWiFi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

}
void  CameraInit()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 40;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  else
  {
    Serial.printf("Camera init OK!!!");
  }

}

void DataJson(String client ,  String TTGate,  String TTLed, String message)
{
  String ChuoiSendWebJson = "";
  ChuoiSendWebJson = "{\"Client\":\"" + String(client) + "\"," +
                     "\"TTGATE\":\"" + String(TTGate) + "\"," +
                     "\"TTLED\":\"" + String(TTLed) + "\"," +
                     "\"MESS\":\"" + String(message) + "\"}";

  //webSocket.sendTXT(ChuoiSendWebJson);

  webSocket.broadcastTXT(ChuoiSendWebJson.c_str());
}
