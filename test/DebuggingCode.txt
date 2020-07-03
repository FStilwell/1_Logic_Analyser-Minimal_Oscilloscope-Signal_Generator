#include <Arduino.h>


//Pin Definitions
#define potPin 14
#define buttonPin 12
#define ch1Pin 16

//Function Prototypes
void ISR_Button();

//Interrupt Variables
volatile bool pressDetected = false;

//***Finite State Machine Setups***
//Events
bool buttonPressed = false;
bool BP_pot0 = false; //Button pressed and pot value 0
bool BP_pot1 = false; //Button pressed and pot value 1
bool BP_pot2 = false;
bool BP_pot3 = false;
bool serialIn0 = false; //Serial input 0
bool serialIn1 = false; //Serial input 1
bool serialIn2 = false;
bool serialIn3 = false;

//States
enum states {
  StartScreen, 
  SelectionMenu,
  OsciMode, 
  FuncGenMode, 
  LogicAnalyserMode, 
  SquareWave, 
  SinWave, 
  TriangleWave,
};

states currentState = StartScreen;

//Constants
const uint8_t debounceTime_ms = 150; //80ms Debounce time

void setup() {
  pinMode(ch1Pin, INPUT);
  pinMode(potPin,INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(buttonPin, ISR_Button, FALLING);

  analogReadResolution(10); //10 bit resolution
  analogWriteResolution(10); //10 bit resolution

  Serial.begin(115200);
}

void loop(){
  /*
  //Testing Serial input
  uint8_t incomingBit = Serial.read();

  if(incomingBit == '0'){serialIn0 = true;}
  else{serialIn0 = false;}

  Serial.print("IncomingBit = "); Serial.print(DEC,incomingBit);
  Serial.print(" serialIn0 = ");  Serial.println(serialIn0); 

  if(serialIn0){Serial.println("Success"); }

  delay(500);
  */

 //Testing button press and pot value
 
}

void ISR_Button(){
  static uint32_t previousMillisButton = millis();
  uint32_t currentMillisButton = millis();

  //Serial.println("Arrived in ISR");
    if ((currentMillisButton - previousMillisButton) >= debounceTime_ms){
      previousMillisButton = currentMillisButton;
      buttonPressed = true;
      //Serial.println(buttonPressed);
    }
    else{
      buttonPressed = false;
    }
}