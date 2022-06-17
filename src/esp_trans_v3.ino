/*





适配esp_quad_v4.0 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!







Add menu steps

  1. add one new char to    char menu[] = {'b', 'n', 'o','o'};
  
    warning - add new chars before 'o'
     
    example
    char menu[] = {'b', 'n', 'w', 'o', 'o'};

  2. setup your "menu_?()" function
    example  
    int menu_wifi(int n, char which){
      if(which != 'w') return -1;

        switch(n){
          case 1:
            // your code
          case 2:
            // your code
        }

      return 1;
    }
    *footnote
      case1 = show display 1
      case2 = show display 2
      case3 = run program once/ begin function. example- radio.begin()
      case4 = loop function

  3. add to   void menu_agent(int n, cahr m)
    example  
    void menu_agent(int num, char m){
    menu_bl(num, m);
    menu_nrf(num, m);
    menu_off(num, m);
    menu_wifi(num, m);
    }



Calibrate your joystick

  uncomment code below in the menu_nrf()
      Serial.print(*(js + 0));
      Serial.print("   ");
      Serial.print(*(js + 1));
      Serial.print("   ");
      Serial.print(*(js + 2));
      Serial.print("   ");
      Serial.println(*(js + 3));
      
  change joystick pins according to your need
    #define j1x 36
    #define j1y 39
    #define j2x 34
    #define j2y 35
  invert output according to your need
    #define invert_lx 0
    #define invert_ly 1
    #define invert_rx 1
    #define invert_ry 0

*/


#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BleGamepad.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SimpleKalmanFilter.h>


// joystick--------------------------------------------------------
int *get_js();
#define j1x 36
#define j1y 39
#define j2x 34
#define j2y 35
#define invert_lx 0
#define invert_ly 1
#define invert_rx 1
#define invert_ry 0
SimpleKalmanFilter filter_lx(5, 5, 0.5);
SimpleKalmanFilter filter_ly(5, 5, 0.5);
SimpleKalmanFilter filter_rx(5, 5, 0.5);
SimpleKalmanFilter filter_ry(5, 5, 0.5);


// v detect--------------------------------------------------------
float *get_v();
#define v1s 33
#define v2s 32


// switch--------------------------------------------------------
#define s1 27
#define s2 14


// gamepad ------------------------------------------------------
BleGamepad bleGamepad;


// rotary encoder---------------------------------------------------
void rotary_onButtonClick();
void IRAM_ATTR readEncoderISR();

#define A_PIN 13
#define B_PIN 5
#define BTT_PIN 4
#define VCC_PIN -1
#define STEPS 8
AiEsp32RotaryEncoder rotaryEncoder =
  AiEsp32RotaryEncoder(A_PIN, B_PIN, BTT_PIN, VCC_PIN, STEPS);


// iic oled--------------------------------------------------------
void refresh_iic();

int menu_bl(int, char);
int menu_off(int, char);
int menu_nrf(int, char);
void menu_agent(int, char);

int oled_en = 1;
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1
#define SCREEN_ADDRESS1 0x3D
#define SCREEN_ADDRESS2 0x3C
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

char menu[] = {'b', 'n', 'o','o'};
int menu_cptr = sizeof(menu)-1;


//nrf--------------------------------------------------------
void serial_blackbox();
int nrf_en = 0;
RF24 radio(25, 26); //ce csn

//nrf 1
struct nrf {
  bool en; //
  bool pos_hold;
  byte joyStick[4]; // x, y, z, t  
};
struct drone_data {
  float voltage;
  byte motorOut[4]; // a, b, c, d
  int cur_gyro[3];  // 当前角速度 x, y, z
  float cur_angle[3]; // 当前角度 x, y, z
};
static struct nrf radioo;
static struct drone_data ack;

//------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // pin mode
  pinMode(j1x, INPUT);
  pinMode(j1y, INPUT);
  pinMode(j2x, INPUT);
  pinMode(j2y, INPUT);
  pinMode(v1s, INPUT);
  pinMode(v2s, INPUT);
  pinMode(s1, INPUT_PULLUP);
  pinMode(s2, INPUT_PULLUP);

  // rotary encoder set
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(0, sizeof(menu) - 1, 1);
  rotaryEncoder.disableAcceleration();

  // display begin
  if (!display1.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS1))for (;;);
  if (!display2.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS2))for (;;);

  // set display brightness
  display1.ssd1306_command(SSD1306_SETCONTRAST);
  display1.ssd1306_command(5);
  display2.ssd1306_command(SSD1306_SETCONTRAST);
  display2.ssd1306_command(5);
  // 开机画面
  display1.display();
  display2.display();
  delay(500);
  // clear
  display1.clearDisplay();
  display2.clearDisplay();
  display1.display();
  display2.display();

  // ...
  display1.clearDisplay();
  display1.setTextColor(SSD1306_WHITE);
  display1.setTextSize(2);
  display1.setCursor(1, 15);
  display1.println(F("press to"));
  display1.println(F("unlock"));
  display1.display();

  // unlock when pressed
  while (not(rotaryEncoder.isEncoderButtonClicked())) {}

  // initiate joystick
  int *js = get_js();

}


//------------------------------------------------------------
void loop() {
  refresh_iic();
  menu_agent(4, menu[menu_cptr]);
  delay(2);
}


//------------------------------------------------------------
//voltage
float *get_v() {
  // read battery voltage
  static float v[2];

  v[0] = analogRead(v1s);
  v[0] = v[0] / 4095 * 3.3;
  v[0] = v[0] * 2;

  v[1] = analogRead(v2s);
  v[1] = v[1] / 4095 * 3.3;
  v[1] = v[1] * 3.125;
  v[1] = v[1] + 0.6;

  return v;
}


// ------------------------------------------------------------
// get joystick
int *get_js() {
  static int vals[4];
  static boolean cali = true;
  static int cali_vals[4] = {0, 0, 0, 0};

  vals[0] = analogRead(j1x);
  vals[1] = analogRead(j1y);
  vals[2] = analogRead(j2x);
  vals[3] = analogRead(j2y);

  vals[0] = map(vals[0], 0, 4095, 0, 255);
  vals[1] = map(vals[1], 0, 4095, 0, 255);
  vals[2] = map(vals[2], 0, 4095, 0, 255);
  vals[3] = map(vals[3], 0, 4095, 0, 255);

  // if your + and - pin reversed
  if (invert_lx == 1) vals[0] = 255 - vals[0];
  if (invert_ly == 1) vals[1] = 255 - vals[1];
  if (invert_rx == 1) vals[2] = 255 - vals[2];
  if (invert_ry == 1) vals[3] = 255 - vals[3];

  // get calibration val
  if (cali) {
    for (int i = 1; i < 4; i++)cali_vals[i] = 128 - vals[i];
    cali = false;
  }
  // calibrate joystick
  for (int i = 0; i < 4; i++) {
    vals[i] = vals[i] + cali_vals[i];
    vals[i] = constrain(vals[i], 0, 255);
  }

  // no description
  vals[0] = filter_lx.updateEstimate(vals[0]);
  vals[1] = filter_ly.updateEstimate(vals[1]);
  vals[2] = filter_rx.updateEstimate(vals[2]);
  vals[3] = filter_ry.updateEstimate(vals[3]);

  return vals;
}


// rotary encoder
void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}

// check if rotary button pressed
void rotary_onButtonClick() {
  static unsigned long lastTimePressed = millis();

  if (rotaryEncoder.isEncoderButtonClicked()
      and (millis() - lastTimePressed > 500)) {
    lastTimePressed = millis();

    // show ok
    display2.clearDisplay();
    display2.setTextSize(3);
    display2.setTextColor(SSD1306_WHITE);
    display2.setCursor(40, 20);
    display2.print(F("ok"));
    display2.display();
    delay(400);

    //disable nrf
    nrf_en = 0;
    // update menu pointer for screen 1
    menu_cptr = rotaryEncoder.readEncoder();

    menu_agent(3, menu[menu_cptr]);

    //if point to off, return -1 to turn off the screen
    if (menu[menu_cptr] == 'o' or 
        menu[menu_cptr] == '\0') oled_en = -1;
  }

}


//------------------------------------------------------------
void refresh_iic() {
  static unsigned long display_timer = millis();
  static unsigned long oled_sleep = 20;

  // recover screen when rotate
  if (rotaryEncoder.encoderChanged()) oled_en = 1;

  // refresh iic evey 50ms
  if (millis() - display_timer > oled_sleep and 
      oled_en == 1) {

    // clear buffer before update
    display1.clearDisplay();
    display2.clearDisplay();

    menu_agent(1, menu[menu_cptr]);
    menu_agent(2, menu[rotaryEncoder.readEncoder()]);
    // show voltage
    display1.setTextSize(1);
    display1.setTextColor(SSD1306_WHITE);
    float *v = get_v();
    display1.setCursor(95, 1);
    display1.print(*(v + 1));
    display1.println(F("v"));

    // update display
    display1.display();
    display2.display();
    display_timer = millis();

    // check if rotary button clicked
    // dont move
    rotary_onButtonClick();

  }

}


int menu_bl(int n, char which) {

  if(which != 'b') return -1;

  switch (n)
  {
    case 1:
      display1.setTextColor(SSD1306_WHITE);
      if (bleGamepad.isConnected()) {
        display1.setTextSize(2);
        display1.setCursor(1, 15);
        display1.println(F("computer"));
        display1.println(F("connected"));
      } else {
        display1.setTextSize(2);
        display1.setCursor(1, 20);
        display1.println(F("no device2"));
      }
      break;

    case 2:
      display2.setTextColor(SSD1306_WHITE);
      display2.setTextSize(3);
      display2.setCursor(40, 20);
      display2.print(F("bl"));
      break;

    case 3:
      bleGamepad.begin(0, 0, true, true, true, true, false, false,
                       false, false, false, false, false, false, false);
      break;

    case 4:
      if (bleGamepad.isConnected()) {
        int *js = get_js();
        bleGamepad.setLeftThumb(*(js + 0), *(js + 1));
        bleGamepad.setRightThumb(*(js + 2), *(js + 3));
      }
      break;
  }

return 1;
}


int menu_off(int n, char which) {

  if(which != 'o' and which != '\0') return -1;

  switch (n) {
    case 1:
      display1.clearDisplay();
      break;

    case 2:
      display2.setTextSize(3);
      display2.setTextColor(SSD1306_WHITE);
      display2.setCursor(40, 20);
      display2.print(F("off"));
      break;

    case 3:
      display1.clearDisplay();
      display2.clearDisplay();
      display1.display();
      display2.display();
      break;
  }

return 1;
}


int menu_nrf(int n, char which) {

  static const byte addresses[][6] = {"00001", "00002"};


  if(which != 'n') return -1;

  switch (n) {
    case 1:
      // lock for safe
      display1.setTextColor(SSD1306_WHITE);
      if (nrf_en == 0) {
        display1.setTextSize(2);
        display1.setCursor(1, 15);
        display1.println(F("<--scroll"));
        display1.println(F("<-down to"));
        display1.println(F("<--unlock"));
      } else {
        display1.setTextSize(1);
        display1.setCursor(1, 1);
        display1.print(F("a"));
        display1.print(ack.motorOut[0]);
        display1.setCursor(1, 18);
        display1.print(F("b"));
        display1.print(ack.motorOut[1]);
        display1.setCursor(1, 36);
        display1.print(F("c"));
        display1.print(ack.motorOut[2]);
        display1.setCursor(1, 54);
        display1.print(F("d"));
        display1.print(ack.motorOut[3]);

        display1.setCursor(38, 1);
        display1.print("v");
        display1.print(ack.voltage);

        display1.setCursor(38, 18);
        display1.print(F("x"));
        display1.print(ack.cur_gyro[0]);
        display1.setCursor(38, 36);
        display1.print(F("y"));
        display1.print(ack.cur_gyro[1]);
        display1.setCursor(38, 54);
        display1.print(F("z"));
        display1.print(ack.cur_gyro[2]);

        display1.setCursor(85, 18);
        display1.print(F("x"));
        display1.print(ack.cur_angle[0]);
        display1.setCursor(85, 36);
        display1.print(F("y"));
        display1.print(ack.cur_angle[1]);
        display1.setCursor(85, 54);
        display1.print(F("z"));
        display1.print(ack.cur_angle[2]);
      }
      break;

    case 2:
      display2.setTextColor(SSD1306_WHITE);
      display2.setTextSize(3);
      display2.setCursor(40, 20);
      display2.print(F("nrf"));
      break;

    case 3:
      // nrf begin
      radioo.en = 0;
      radioo.pos_hold = 0;
      radio.begin();
      radio.enableDynamicPayloads();
      radio.enableAckPayload();
      radio.openReadingPipe(1, addresses[0]);
      radio.openWritingPipe(addresses[1]);
      radio.stopListening();
      radio.setPALevel(RF24_PA_MAX);
      break;

    case 4:
      static unsigned long nrf_recieve_time;
      //7777777777777777777777777777777777777777777777777777777777
      int *js = get_js();
      // check switch status
      if (not(digitalRead(s1))) radioo.pos_hold = 1;
      else radioo.pos_hold = 0;
      if (not(digitalRead(s2))) radioo.en = 1;
      else radioo.en = 0;

      // lock nrf for safe
      if (*(js + 0) == 0 and radioo.en == false) nrf_en = 1;

      if (nrf_en != 1) break;

      byte xyzt[4];
      xyzt[0] = *(js + 2);
      xyzt[1] = *(js + 3);
      xyzt[2] = *(js + 1);
      xyzt[3] = *(js + 0);
      for (int i = 0; i < 4; i++) radioo.joyStick[i] = xyzt[i];
      /*
      Serial.print(xyzt[0]);
      Serial.print("   ");
      Serial.print(xyzt[1]);
      Serial.print("   ");
      Serial.print(xyzt[2]);
      Serial.print("   ");
      Serial.print(xyzt[3]);
      Serial.print("   ");
      Serial.print(radioo.pos_hold);     
      Serial.print("   ");
      Serial.println(radioo.en);           
      */
      // nrf write
      bool report = radio.write(&radioo, sizeof(radioo));
      if (report){
        if (radio.available()) radio.read(&ack, sizeof(ack));
        nrf_recieve_time = millis();
      }
      // if disconnected
      if (millis() - nrf_recieve_time > 500) {
        ack.voltage = 0.0;
        for (int i=0; i<4; i++) ack.motorOut[i] = 0;
        for (int i=0; i<3; i++)ack.cur_gyro[i] = 0;
        for (int i=0; i<3; i++)ack.cur_angle[i] = 0.0;
      }else serial_blackbox();
      break;
  }
return 1;
}


void serial_blackbox(){
  /*
  Serial.print(ack.cur_angle[0], 2);
  Serial.print(";");
  Serial.print(ack.cur_angle[1], 2);
  Serial.print(";");
  Serial.print(ack.cur_angle[2], 2);
  Serial.print(";");
  Serial.print(ack.cur_gyro[0]);
  Serial.print(";");
  Serial.print(ack.cur_gyro[1]);
  Serial.print(";");
  Serial.print(ack.cur_gyro[2]);
  Serial.print(";");
  Serial.print(ack.motorOut[0]);
  Serial.print(";");
  Serial.print(ack.motorOut[1]);
  Serial.print(";");
  Serial.print(ack.motorOut[2]);
  Serial.print(";");
  Serial.println(ack.motorOut[3]);
*/


  Serial.println(ack.cur_gyro[0]);
  //Serial.print(";");
  //Serial.print(ack.cur_gyro[1]);
  //Serial.print(";");
  //Serial.println(ack.cur_gyro[2]);

}


void menu_agent(int num, char m){
menu_bl(num, m);
menu_nrf(num, m);
menu_off(num, m);
}
