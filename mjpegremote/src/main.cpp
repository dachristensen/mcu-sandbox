#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebSocketsClient.h>
#include <Adafruit_SSD1351.h>
#include "JPEGDEC.h"
#include <OneButton.h>

// Wifi info
#define WIFI_SSID "svoidlan"
#define WIFI_PASSWORD "daniel.l.christensen"

// Connection info
#define WEBSOCKET_HOST "192.168.0.197"
#define WEBSOCKET_PORT 8000

// JOYSTICK pins
#define JS_RIGHT_X 39
#define JS_RIGHT_Y 36
#define JS_RIGHT_BTN 34
#define JS_LEFT_X 32
#define JS_LEFT_Y 35
#define JS_LEFT_BTN 13
#define SWITCH_DRIVE 25
#define SWITCH_CAMERA 26
#define JS_ADC_RANGE 4096

// OLED pins
#define OLED_WIDTH 128
#define OLED_HEIGHT 128
#define OLED_CS   5
#define OLED_RST  16
#define OLED_DC   17

// Color definitions
#define	BLACK     0x0000
#define	BLUE      0x001F
#define	RED       0xF800
#define	GREEN     0x07E0
#define CYAN      0x07FF
#define MAGENTA   0xF81F
#define YELLOW    0xFFE0  
#define WHITE     0xFFFF

// Globals
WiFiMulti wifimgr;
WebSocketsClient ws;
Adafruit_SSD1351 oled = Adafruit_SSD1351(OLED_WIDTH,OLED_HEIGHT,&SPI,OLED_CS,OLED_DC,OLED_RST);
JPEGDEC jpeg;
bool connected = false;
unsigned long lastread = 0;
static const int READ_DELAY = 200;
OneButton js_left_btn(JS_LEFT_BTN);
OneButton js_right_btn(JS_RIGHT_BTN);

/**
 *  Draw the decoded jpeg to the screen as an RGB bitmap. Called by the 
 *   JPEGDEC as a result calling the decode method.
 */
void draw(JPEGDRAW *pDraw)
{
  oled.drawRGBBitmap(pDraw->x,pDraw->y,pDraw->pPixels,pDraw->iWidth,pDraw->iHeight);
}

/**
 * Respond to web socket events. Called by the WebSocketClient whenever
 *  an event is received.
 */ 
void onWsEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.printf("Disconnected!\n");
    oled.fillScreen(0);
    oled.setCursor(0,0);
    oled.println("Disconnected");
    connected = false;
    break;
  case WStype_CONNECTED:
    Serial.printf("Connected to url: %s\n", payload);
    oled.println("Connected");
    connected = true;
    break;
  case WStype_BIN:
    jpeg.openRAM((uint8_t*)payload, length, draw);
    jpeg.decode(0,0,0);
    break;
  case WStype_TEXT:
    Serial.printf("Unexpected TEXT message");
    break;
  }
}

void onJoystickClick(void *param)
{
  boolean drive = digitalRead(SWITCH_DRIVE) == HIGH;
  if(param == reinterpret_cast<void*>(JS_LEFT_BTN))
  {
    Serial.println("Left Clicked");
    if(connected && !drive)
    {
      ws.sendTXT("{\"pan\":\"home\",\"tilt\":\"home\"}");
    }
  }
  else if(param == reinterpret_cast<void*>(JS_RIGHT_BTN))
  {
    Serial.println("Right Clicked");
  }
}

void setup() {
  // Init Serial
  Serial.begin(115200);
  
  //Init GPIO
  pinMode(OLED_CS, OUTPUT);
  pinMode(OLED_RST, OUTPUT);
  pinMode(OLED_DC, OUTPUT);
  pinMode(JS_RIGHT_X, INPUT);
  pinMode(JS_RIGHT_Y, INPUT);
  // pinMode(JS_RIGHT_BTN, INPUT);
  pinMode(JS_LEFT_X, INPUT);
  pinMode(JS_LEFT_Y, INPUT);
  // pinMode(JS_LEFT_BTN, INPUT);
  pinMode(SWITCH_DRIVE, INPUT);
  pinMode(SWITCH_CAMERA, INPUT);

  // Init joystick buttons
  js_left_btn.attachClick(onJoystickClick, (void*)JS_LEFT_BTN);
  js_right_btn.attachClick(onJoystickClick, (void*)JS_RIGHT_BTN);

  //Init SPI
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(0);
  SPI.begin();

  // Init OLED display
  oled.begin();
  oled.fillScreen(0);
  oled.setTextColor(BLUE);
  oled.setCursor(0,0);
  oled.println("Connecting to Wifi");

  // Init Wifi
  wifimgr.addAP(WIFI_SSID, WIFI_PASSWORD);
  while (wifimgr.run() != WL_CONNECTED)
  {
    delay(100);
  }

  // Print wifi connection details to serial
  Serial.println("Wifi Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP().toString());

  // Print wifi connection details to display
  oled.print("IP: ");
  oled.println(WiFi.localIP().toString());
  oled.println();

  // Init web socket
  oled.println("Connecting to:");
  String wsurl = WEBSOCKET_HOST;
  wsurl += ":";
  wsurl += WEBSOCKET_PORT;
  oled.println(wsurl);
  ws.begin(WEBSOCKET_HOST, WEBSOCKET_PORT);
  ws.onEvent(onWsEvent);
  ws.setReconnectInterval(5000);
  ws.enableHeartbeat(15000, 3000, 2);
}

void loop() {
  ws.loop();
  js_left_btn.tick();
  js_right_btn.tick();

  long now = millis();
  if(connected && ((now - lastread) > READ_DELAY))
  {
    int lx = JS_ADC_RANGE - analogRead(JS_LEFT_X);
    int ly = JS_ADC_RANGE - analogRead(JS_LEFT_Y);
    int rx = JS_ADC_RANGE - analogRead(JS_RIGHT_X);
    int ry = JS_ADC_RANGE - analogRead(JS_RIGHT_Y);
    String json = "";
    if(digitalRead(SWITCH_DRIVE) == HIGH){
      float accel = (ly * 2.0)/JS_ADC_RANGE - 1;
      float turn = (rx * 2.0)/JS_ADC_RANGE - 1;
      json = "{\"accel\":";
      json += accel;
      json += ",\"turn\":";
      json += turn;
      json += "}";
    } else {
      float accel = (ry * 2.0)/JS_ADC_RANGE - 1;
      float turn = (rx * 2.0)/JS_ADC_RANGE - 1;
      float pan = (lx * 2.0)/JS_ADC_RANGE - 1;
      float tilt = (ly * 2.0)/JS_ADC_RANGE - 1;
      json = "{\"accel\":";
      json += accel;
      json += ",\"turn\":";
      json += turn;
      json += ",";
      json += "\"pan\":";
      json += pan;
      json += ",\"tilt\":";
      json += tilt;
      json += "}";
    }
    ws.sendTXT(json);
    lastread = now;
  }
}