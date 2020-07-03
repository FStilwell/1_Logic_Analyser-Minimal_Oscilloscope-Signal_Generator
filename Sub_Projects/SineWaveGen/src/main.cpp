#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//Pin Definitions
#define ch1Pin 16
#define DAPin A14

//Function Prototypes
void simpleOscilloscope();

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Global Variables
const uint32_t plotWidthOffset = 1; //1 pixel from left
const uint32_t plotWidth = SCREEN_WIDTH - plotWidthOffset - 1; 
const uint32_t plotHeightOffset = 7; //Offset to allow for text at top of screen
const uint32_t plotHeight = SCREEN_HEIGHT-1 - plotHeightOffset ;

const uint32_t scanTime_us = 10000;
const uint32_t sinSigRange = 1023; //Signal range (0-1023)

void setup() {
 if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();

  analogWriteResolution(10);
  analogReadResolution(10);

  //PinModes
  pinMode(ch1Pin, INPUT);
  pinMode(DAPin, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  //Declare Timing Variables
 static uint32_t previousTime_us = 0; //Declares only once
 uint32_t currentTime_us = micros();


  if ((currentTime_us - previousTime_us) >= scanTime_us){
    //============= Generating Signal =============
      //X signal Values
      static uint32_t XSigVal = 0;
      if(XSigVal >= 0 && XSigVal <= plotWidth){XSigVal++;} //Configure counter
      else{XSigVal = 0; display.clearDisplay();}
      //Y signal Values
      static uint32_t YSigVal = 0; //Current signal value
      uint32_t XsinVal = map(XSigVal,0,plotWidth,0,62831); //Map plot width to 2*Pi*10000
      YSigVal = sin((double)XsinVal/10000) * sinSigRange/2 + sinSigRange/2;

    //Output D/A signal
    analogWrite(DAPin, YSigVal);

    //Plot Values
    simpleOscilloscope();

  }
}

void simpleOscilloscope(){
    //X Draw Values
    static uint32_t currXDrawVal = 0; //Declares only once
    uint32_t prevXDrawVal = currXDrawVal-1;
    if(currXDrawVal >= 0 && currXDrawVal <= plotWidth){currXDrawVal++;} //Configure counter
    else{currXDrawVal = prevXDrawVal = 0; display.clearDisplay();}
    

    //Channel 1 A/D reading and mV mapping
    uint32_t potVal = analogRead(ch1Pin); 
    uint32_t volts_mV = map(potVal,0,1023,0,3300);

    //Y Draw Values
    static uint32_t prevYDrawVal = 0;
    static uint32_t currYDrawVal = 0;
    prevYDrawVal = currYDrawVal; //y value from previous cycle
    currYDrawVal = plotHeight - map(analogRead(ch1Pin),0, 1023, 0, plotHeight);
    currYDrawVal = currYDrawVal + plotHeightOffset;

    //===================== OLED Screen Plotting =====================
    //Draw X Axis
    display.drawLine(0,SCREEN_HEIGHT-1,SCREEN_WIDTH-1,SCREEN_HEIGHT-1, WHITE);
    //Draw Y Axis
    display.drawLine(0, plotHeightOffset, 0, SCREEN_HEIGHT-1, WHITE);
    
    //Display Value
    static int16_t displayValXPos = (SCREEN_WIDTH-1)/2 + 2;  //Start at 1/3 of screen
    display.fillRect(displayValXPos, 0, displayValXPos -2, plotHeightOffset, BLACK);
    display.setTextSize(1); display.setTextColor(WHITE); // Draw white text, size 1
    display.setCursor(displayValXPos, 0);                // Start at top middle
    display.print(("Y = "));
    display.print((volts_mV));
    display.println(("mV"));

    //Plot Values
    display.drawLine(prevXDrawVal, prevYDrawVal, currXDrawVal, currYDrawVal, WHITE);
    Serial.println(volts_mV);
    
    //Update Screen
    display.display();  
}
