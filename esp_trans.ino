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
#define j2x 35
#define j2y 34
#define invert_lx 0
#define invert_ly 0
#define invert_rx 1
#define invert_ry 1
SimpleKalmanFilter filter_lx(5, 5, 0.5);
SimpleKalmanFilter filter_ly(5, 5, 0.5);
SimpleKalmanFilter filter_rx(5, 5, 0.5);
SimpleKalmanFilter filter_ry(5, 5, 0.5);


// v detect--------------------------------------------------------
float *get_v();
#define v1s 33
#define v2s 32


// rotary encoder---------------------------------------------------
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
void show_ok();
void set_flags();
void show_voltage();
void clean_display_both();
void show_nrf();
void show_bl();

char menu[4] = {'b', 'n', 'o'};

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1
#define SCREEN_ADDRESS1 0x3D
#define SCREEN_ADDRESS2 0x3C
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// switch--------------------------------------------------------
void send_switch();
#define s1 27
#define s2 14


// gamepad ------------------------------------------------------
BleGamepad bleGamepad;
void send_data_bl();


//nrf------------------------------------------------------------
void send_data_nrf();
void serial_nrf();

RF24 radio(25, 26); //ce csn
const byte addresses[][6] = {"00001", "00002"};


// data------------------------------------------------------------
struct nrf {
  byte pwm[4];
  float rpid[3];
  float rpidz[3];
  byte en;
};
struct drone_data {
  float voltages[3];
  byte motorOut[4];
  float cAngles[3];
};
struct nrf radioo;
struct drone_data ack;

// pid
float Kp  = 1.8; //2.5
float Ki  = 2.0; // /1000
float Kd  = 70.0;
float Kpz  = 4.5;
float Kiz  = 3.0; // /1000
float Kdz  = 50.0;


// special
int rotary_onButtonClick(char);
boolean bl_flag = 0;
boolean nrf_flag = 0;
boolean nrf_en = 0;

//------------------------------------------------------------
void setup() {
  Serial.begin(250000);

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
  bool circleValues = 1;
  rotaryEncoder.setBoundaries(0, 3, circleValues);
  rotaryEncoder.disableAcceleration();

  // display begin
  if (!display1.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS1))for (;;);
  if (!display2.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS2))for (;;);

  // set display brightness
  display1.ssd1306_command(SSD1306_SETCONTRAST);
  display1.ssd1306_command(5);
  // 开机画面
  display1.display();
  display2.ssd1306_command(SSD1306_SETCONTRAST);
  display2.ssd1306_command(5);
  // 开机画面
  display2.display();
  delay(500);
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

  // bl begin
  /*bleGamepad.begin(0, 0, true, true, true, true, false, false,
                   false, false, false, false, false, false, false);
  */
  // nrf begin
  radioo.rpid[0] = Kp;
  radioo.rpid[1] = Ki;
  radioo.rpid[2] = Kd;
  radioo.rpidz[0] = Kpz;
  radioo.rpidz[1] = Kiz;
  radioo.rpidz[2] = Kdz;
  radioo.en = 0;
  radio.begin();
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openReadingPipe(1, addresses[0]);
  radio.openWritingPipe(addresses[1]);
  radio.stopListening();
  radio.setPALevel(RF24_PA_MAX);
}


//------------------------------------------------------------
void loop() {

  //int *js = get_js();
  refresh_iic();
  if (bl_flag) send_data_bl();

  if (nrf_flag) send_data_nrf();
  if (nrf_flag) serial_nrf();

  delay(5);
}



//------------------------------------------------------------
//voltage
float *get_v() {
  // read battery voltage
  // change accroding to your voltage devider
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
// joystick
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

  // calibrate joystick
  if (cali) {
    for (int i = 1; i < 4; i++)cali_vals[i] = 128 - vals[i];
    cali = false;
  }
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


//------------------------------------------------------------
// rotary encoder
void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}

int rotary_onButtonClick(char c) {
  static unsigned long lastTimePressed = millis();
  //disable nrf_en after each press for safety
  // return -1 to turn off screen
  if (rotaryEncoder.isEncoderButtonClicked()) {
    if (millis() - lastTimePressed > 500) {
      nrf_en = 0;
      // show ok when pressed
      show_ok();
      lastTimePressed = millis();

      switch (c) {
        case 'o':
          delay(500);
          clean_display_both();
          set_flags('o');
          return -1;
          break;

        case '\0':
          delay(500);
          set_flags('o');
          return -1;
          break;

        case 'n':
          set_flags('n');
          break;

        case 'b':
          set_flags('b');
          break;
      }

      return 500; // return 500 if pressed for ok
    }

  }
  return 50; // return 50 for nothing happens

}


//------------------------------------------------------------
void refresh_iic() {
  static unsigned long display_timer = millis();
  static unsigned long oled_sleep = 50;
  // recover screen when rotate
  if (rotaryEncoder.encoderChanged()) oled_sleep = 50;
  // refresh
  if (millis() - display_timer > oled_sleep and oled_sleep != -1) {
    // reset oled_sleep after show_ok
    if (oled_sleep == 500)oled_sleep = 50;

    // print voltage
    show_voltage();
    // show nrf
    if (nrf_flag) show_nrf();
    // show bl
    if (bl_flag) show_bl();
    display1.display();

    // print menu on display2
    display2.clearDisplay();
    display2.setTextSize(3);
    display2.setTextColor(SSD1306_WHITE);
    switch (menu[rotaryEncoder.readEncoder()]) {
      case 'n':
        display2.setCursor(40, 20);
        display2.print(F("nrf"));
        oled_sleep = rotary_onButtonClick('n');
        break;
      case 'b':
        display2.setCursor(40, 20);
        display2.print(F("bl"));
        oled_sleep = rotary_onButtonClick('b');
        break;
      case 'o':
        display2.setCursor(40, 20);
        display2.print(F("off"));
        oled_sleep = rotary_onButtonClick('o');
        break;
      case '\0':
        display2.setCursor(40, 20);
        display2.print(F("off"));
        oled_sleep = rotary_onButtonClick('\0');
        break;
    }
    display2.display();

    display_timer = millis();
  }
}


//------------------------------------------------------------
void send_data_bl() {
  if (bleGamepad.isConnected())
  {
    int *js = get_js();
    bleGamepad.setLeftThumb(*(js + 0), *(js + 1));
    bleGamepad.setRightThumb(*(js + 2), *(js + 3));
  }
}


//------------------------------------------------------------
void send_data_nrf() {
  static unsigned long nrf_recieve_timeout;
  int *js = get_js();
  // lock nrf for safe
  if (*(js + 0) == 0) nrf_en = 1;

  if (nrf_en) {
    byte xyzt[4];
    xyzt[0] = *(js + 2);
    xyzt[1] = *(js + 3);
    xyzt[2] = *(js + 1);
    xyzt[3] = *(js + 0);
    for (int i = 0; i < 4; i++) radioo.pwm[i] = xyzt[i];

    // check switch status
    if (not(digitalRead(s2))) {
      radioo.en = 1;
    } else {
      radioo.en = 0;
    }

    // nrf write
    bool report = radio.write(&radioo, sizeof(radioo));
    if (report) {
      if (radio.available()) radio.read(&ack, sizeof(ack));
      nrf_recieve_timeout = millis();
    }
    // if disconnected
    if (millis() - nrf_recieve_timeout > 500) {
      for (int i = 0; i < 3; i++) {
        ack.cAngles[i] = 0.0;
        ack.voltages[i] = 0.0;
        ack.motorOut[i] = 0;
      }
      ack.motorOut[3] = 0;
    }
  }

}


//------------------------------------------------------------
// main purpose is to tune pid without flashing esp32
// but need to flash esp32 for your final pid settings
void serial_nrf() {
  if (Serial.available() > 0) {
    String spid  = Serial.readString();
    int slen = spid.length();
    switch (spid[0]) {
      case 'p':
        spid = spid.substring(1, slen - 1);
        radioo.rpid[0] = spid.toFloat();
        break;
      case 'i':
        spid = spid.substring(1, slen - 1);
        radioo.rpid[1] = spid.toFloat();
        break;
      case 'd':
        spid = spid.substring(1, slen - 1);
        radioo.rpid[2] = spid.toFloat();
        break;
      case 'a':
        spid = spid.substring(1, slen - 1);
        radioo.rpidz[0] = spid.toFloat();
        break;
      case 'b':
        spid = spid.substring(1, slen - 1);
        radioo.rpidz[1] = spid.toFloat();
        break;
      case 'c':
        spid = spid.substring(1, slen - 1);
        radioo.rpidz[2] = spid.toFloat();
        break;
    }
  }

  Serial.print("p:");
  Serial.print(radioo.rpid[0], 3);
  Serial.print("  i:");
  Serial.print(radioo.rpid[1], 3);
  Serial.print("  d:");
  Serial.print(radioo.rpid[2], 3);

  Serial.print("     pz:");
  Serial.print(radioo.rpidz[0], 3);
  Serial.print("  iz:");
  Serial.print(radioo.rpidz[1], 3);
  Serial.print("  dz:");
  Serial.print(radioo.rpidz[2], 3);

  Serial.print("      x:");
  Serial.print(ack.cAngles[0], 2);
  Serial.print("   y:");
  Serial.print(ack.cAngles[1], 2);
  Serial.print("   z:");
  Serial.print(ack.cAngles[2], 2);

  Serial.print("     v1:");
  Serial.print(ack.voltages[0], 2);
  Serial.print("   v2:");
  Serial.print(ack.voltages[1], 2);
  Serial.print("   v3:");
  Serial.print(ack.voltages[2], 2);

  Serial.print("     B:" + String(ack.motorOut[1]) + "  A:" + String(ack.motorOut[0]));
  Serial.println("    D:" + String(ack.motorOut[3]) + "  C:" + String(ack.motorOut[2]));

}



void clean_display_both() {
  display1.clearDisplay();
  display2.clearDisplay();
  display1.display();
  display2.display();
}

void show_ok() {
  display2.clearDisplay();
  display2.setTextSize(3);
  display2.setTextColor(SSD1306_WHITE);
  display2.setCursor(40, 20);
  display2.print(F("ok"));
  display2.display();
}

void set_flags(char n) {
  if (n == 'b') {
    nrf_flag = 0;
    bl_flag = 1;
    bleGamepad.begin(0, 0, true, true, true, true, false, false,
                     false, false, false, false, false, false, false);
  }
  if (n == 'n') {
    nrf_flag = 1;
    bl_flag = 0;
    bleGamepad.end();
  }
  if (n == 'o') {
    clean_display_both();
    nrf_flag = 0;
    bl_flag = 0;
    bleGamepad.end();
  }
}

void show_voltage() {
  display1.clearDisplay();
  display1.setTextSize(1);
  display1.setTextColor(SSD1306_WHITE);
  float *v = get_v();
  display1.setCursor(95, 1);
  display1.print(*(v + 1));
  display1.println(F("v"));
}

void show_nrf() {
  // lock for safe
  if (nrf_en == 0) {
    display1.setTextSize(2);
    display1.setCursor(1, 15);
    display1.println(F("<--scroll"));
    display1.println(F("<-down to"));
    display1.println(F("<--unlock"));
  } else {
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

    display1.setTextSize(1);
    display1.setCursor(38, 18);
    display1.print(ack.voltages[0]);
    display1.print(F("v"));
    display1.setCursor(38, 36);
    display1.print(ack.voltages[1]);
    display1.print(F("v"));
    display1.setCursor(38, 54);
    display1.print(ack.voltages[2]);
    display1.print(F("v"));

    display1.setCursor(85, 18);
    display1.print(F("x"));
    display1.print(ack.cAngles[0]);
    display1.setCursor(85, 36);
    display1.print(F("y"));
    display1.print(ack.cAngles[1]);
    display1.setCursor(85, 54);
    display1.print(F("z"));
    display1.print(ack.cAngles[2]);
  }
}

void show_bl() {
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
}

/*
    Serial.print(*(js + 0));
    Serial.print("   ");
    Serial.print(*(js + 1));
    Serial.print("   ");
    Serial.print(*(js + 2));
    Serial.print("   ");
    Serial.println(*(js + 3));
*/
