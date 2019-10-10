#include <SoftwareSerial.h>
#include "inc/tfmini.h"
#include "inc/graphics.h"
#include "inc/pin_assignments.h"

SoftwareSerial mySerial(19, 18);      // Uno RX (TFMINI TX), Uno TX (TFMINI RX)
TFMini tfmini;

#define INTERVAL 100 // units of milliseconds
#define TIMEOUT 100 // units of intervals

#define PERFECT_DISTANCE 120 // units of centimeters
#define FAR_DISTANCE 170 // units of centimeters
#define OFF_DISTANCE 350 // units of centimeters

enum STATE
{
  STATE_OFF,
  STATE_STOP,
  STATE_GOOD,
  STATE_BAR,
} state;

uint16_t dist = 0;
uint16_t dist_last = 0;

#define COLOR_RED 1
#define COLOR_GREEN 0

bool color;

void setup() {
  randomSeed(analogRead(0));
  
  pinMode(GND1, OUTPUT);
  pinMode(GND2, OUTPUT);
  pinMode(GND3, OUTPUT);
  pinMode(GND4, OUTPUT);
  pinMode(GND5, OUTPUT);
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(G1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(G2, OUTPUT);
  pinMode(LAT, OUTPUT);
  pinMode(CLK, OUTPUT);

  digitalWrite(GND1, LOW);
  digitalWrite(GND2, LOW);
  digitalWrite(GND3, LOW);
  digitalWrite(GND4, LOW);
  digitalWrite(GND5, LOW);
  digitalWrite(EN, LOW); // LOW is ON

  digitalWrite(CLK, LOW);
  digitalWrite(LAT, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
  digitalWrite(G1, LOW);
  digitalWrite(G2, LOW);
  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(C, LOW);
  digitalWrite(D, LOW);

  Serial.begin(115200); // console

  Serial.println ("Initializing...");

  Serial1.begin(TFMINI_BAUDRATE);

  tfmini.begin(&Serial1);
  delay(100);
  
  tfmini.setSingleScanMode(); 
}


void setRow(unsigned row)
{
  digitalWrite(A, 0x1 & (row >> 0));
  digitalWrite(B, 0x1 & (row >> 1));
  digitalWrite(C, 0x1 & (row >> 2));
  digitalWrite(D, 0x1 & (row >> 3));
}


void drawFrame(const uint8_t buf[], bool color)
{
  int pix_top, pix_bot;
  unsigned idx = 0;
  unsigned x1, x2;

  for(unsigned row = 0; row < 16; row++) {
    __asm__("nop");
    digitalWrite(LAT, LOW);
    setRow(row-1);
    x1 = 8 * row;
    x2 = 8 * (16 + row);
    for(int col = 0; col < 8; col++) {
      for(int pan = 7; pan >= 0; pan--) {
        __asm__("nop");
        digitalWrite(CLK, LOW);
        idx = x1 + col;
        pix_top = 0x1 & (buf[idx] >> pan);
        idx = x2 + col;
        pix_bot = 0x1 & (buf[idx] >> pan);
        digitalWrite(color?R1:G1, pix_top);
        digitalWrite(color?R2:G2, pix_bot);
        __asm__("nop");
        digitalWrite(CLK, HIGH);
      }
    }
    __asm__("nop");
    digitalWrite(LAT, HIGH);
  }
}



uint8_t frame_buf[256] = {0};

void assembleFrame(uint8_t* buf, FRAME_HALF buf1, FRAME_QUARTER buf2, FRAME_QUARTER buf3)
{
  unsigned x, y;
  
  // put image on left
  for( x = 0; x < 4; x++ ) {
    for( y = 0; y < 32; y++ ) {
      buf[8 * y + x] = buf1[4 * y + x];
    }
  }

  // draw first number
  for( x = 0; x < 2; x++ ) {
    for( y = 0; y < 32; y++ ) {
      buf[8 * y + x + 4] = buf2[2 * y + x];
    }
  }

  // draw second number
  for( x = 0; x < 2; x++ ) {
    for( y = 0; y < 32; y++ ) {
      buf[8 * y + x + 6] = buf3[2 * y + x];
    }
  }
}

const uint8_t *const numbers[11] = 
{
  number0_map,
  number1_map,
  number2_map,
  number3_map,
  number4_map,
  number5_map,
  number6_map,
  number7_map,
  number8_map,
  number9_map,
  blank_quarter_map
};

const uint8_t *const bars[6] =
{
  bar0_map,
  bar1_map,
  bar2_map,
  bar3_map,
  bar4_map,
  bar5_map
};

unsigned bar_idx = 0;
int16_t meas_last = 0;
unsigned timeout = 0;

void changeState()
{
  unsigned idx_left;
  unsigned idx_right;
  int16_t meas;

  // timeout if measurement hasn't changed in the last TIMEOUT number of INTERVALS
  if( dist_last == (dist/10) ) {
    if( timeout > TIMEOUT ) digitalWrite(EN, HIGH); // HIGH is OFF
    timeout++;
  }
  else {
    digitalWrite(EN, LOW); // LOW is ON
    timeout = 0;
  }
  dist_last = dist / 10;

  // measurement to display on screen
  meas = abs(((int16_t)dist) - PERFECT_DISTANCE);
  if( meas > 99 ) {
    idx_left = 10;  // the blank symbol
    idx_right = 10; // the blank symbol
  }
  else if( meas < 0 )
  {
    idx_left = 0;
    idx_right = 0;
  }
  else {
    idx_left = meas / 10;
    idx_right = meas % 10;
  }
  
  switch(state)
  {
    case STATE_OFF:
      assembleFrame(frame_buf, blank_half_map, blank_quarter_map, blank_quarter_map);
      if( dist < OFF_DISTANCE ) state = STATE_BAR;
      return;
      
    case STATE_GOOD:
      assembleFrame(frame_buf, thumb_map, numbers[idx_left], numbers[idx_right]);
      color = COLOR_GREEN;
      if( dist < PERFECT_DISTANCE ) state = STATE_STOP;
      else if( dist > FAR_DISTANCE ) state = STATE_BAR;
      return;
      
    case STATE_STOP:
      assembleFrame(frame_buf, stop_map, numbers[idx_left], numbers[idx_right]);
      color = COLOR_RED;
      if( dist > PERFECT_DISTANCE ) state = STATE_GOOD;
      return;
      
    case STATE_BAR:
      assembleFrame(frame_buf, bars[(bar_idx++)%6], numbers[idx_left], numbers[idx_right]);
      color = COLOR_RED;
      if( dist < FAR_DISTANCE ) {
        state = STATE_GOOD;
      }
      else if( dist > OFF_DISTANCE ) {
        state = STATE_OFF;
      }
      return;
      
    default:
      return;
  }
}

long previousMillis = 0;

void loop() {
  unsigned long currentMillis;

  state = STATE_OFF;

  
  while(1) {
    currentMillis = millis();
    if(currentMillis - previousMillis > INTERVAL) {
      previousMillis = currentMillis; 
      changeState();
      
      tfmini.externalTrigger();
      dist = tfmini.getDistance();
    }
    drawFrame(frame_buf, color);
  }
}
