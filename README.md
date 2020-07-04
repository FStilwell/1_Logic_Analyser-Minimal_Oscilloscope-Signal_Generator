# Assignment 1: “Logic Analyzer + Minimal Oscilloscope + Signal Generator" Report

Embed assignment coversheet here

## 1. Abstract

## 2. Introduction

The aim of this project was to design and construct a prototype of a system with the functionality of a minimal oscilloscope, signal generator with square wave, triangle wave and sine wave output capability, and a logic analyser.

The requirements of the project are as follows:

1. One channel oscilloscope (0V - 3.3V) using the uC ADC peripheral
2. One channel function generator using the uC DAC with the option to select between at least a square, triangle and sin signal)
3. One channel logic analyzer, that allows for decoding of a 9600 baud serial frame
4. When acting in one of the above-mentioned modes, the relevant signals should be streamed out in “real-time” via UART(-> USB) to a connected PC and visualized using a serial plotter and in parallel displayed on the provided OLED display.
5. It should be possible to control the different functions(modes) via connected push-buttons and in parallel via commands send from the PC to the uC via USB(->UART)
6. The system should be structured and implemented as a finite state machine

The protype was designed and constructed on a breadboard using the following components:

1. Teensy 3.2 Microcontroller development board
2. SDD1306 I2C controlled OLED Display
3. 10kOhm potentiometer
4. Pushbutton
5. Various jumper wires

The program was intially created in several smaller programs, each with dedicated functions, then they were integrated in a main program.

## 3. Methods

### 3.1 Prototype Overview

Most of the prototype was pre-assembled from a previous prototype. A second breadboard was added to include the OLED screen.

#### Overview Image

The annotated overview image is seen in figure 1 below.

![Prototype overview](Project_Media/Images/Prototype_overview_annotated.png)
*Figure 1: Annotated Prototype Overview*

#### Schematic

The circuit schematic in figure 2 was created using Autodesk Eagle. The files for which can be found [here](Project_Media/Eagle_files).

![Circuit schematic](Project_Media/Images/Circuit_schematic.PNG)
*Figure 2: Circuit schematic*

### Operation

This system can be operated simultaneosly using PC USB to UART character inputs or the system's pushbutton and potentiometer as a cursor control.

### 3.2 Finite State Machine Design

A Moore Finite state machine was used for the program as the system can be sectioned into varous descrete modes, with its output actions only determined by its current state.

#### States

Firstly, the states were defined as follows:

* **Start Screen**: The user can be greeted by a start screen where they can choose start when they are ready.
* **Selection Menu**: Provide a menu on the OLED screen to choose which mode the user wants.
* **Osci. Mode**: Minimal oscilloscope mode, where a voltage signal can be read and visualised on a PC with a serial plotter and on the prototype through the SDD1306 OLED screen in real time.
* **Func.Gen. Mode**: Function generator mode, where the user can choose which function they want.
* **Logic An. Mode**: Logic analyser mode, where a 9600 baud dataframe can be read and interpreted.
* **Square wave**: Produce a square wave using the Teensy3.2's digital to analogue converter.
* **Sine wave**: Produce a sine wave using the Teensy3.2's digital to analogue converter.
* **Triangle Wave**: Produce a Triangle wave using the Teensy3.2's digital to analogue converter.

#### Events

Utilising the pushbutton, 10Kohn potentiometer, and USB to UART serial interface, the following event conditions could be defined:

* Button_Pressed
* Button_Pressed_and_potVal0
* Button_Pressed_and_potVal1
* Button_Pressed_and_potVal2
* Button_Pressed_and_potVal3
* Serial_input_1
* Serial_input_2
* Serial_input_3

#### Actions

The actions are likely to be complex, so the following actions can be taken as general concepts for functions:

* Square wave signal
* Sine wave signal
* Triangle wave signal
* Read channel signal
* Analyse logic
* OLED waveform display
* OLED text display
* Serial plot waveform display

#### State Transition Diagram

The state transition diagram can be seen in figure 3 below. Full PDF version can be found [here](Project_Media/STD_diagram.pdf). This diagram utilizes all states, events, and actions outlined previously. The diagram is busier than it needs to be as each transition is shown as serparate event arrows. This could have been simplified by combining each event with "or" statements.

![State Transition Diagram](Project_Media/Images/STD_image.PNG)
*Figure 3: State Transition Diagram*

#### 3.2.1 Finite State Machine Implementation

The implementation before any actions were added can be seen below.

#### Setups

    //***Finite State Machine Setups***
    //Events
    volatile bool buttonPressed = false;
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

The "enum" variable type is useful for defining the states as it can enumerate a comma-separated list. Each enumerator in the list can be called as its name, but it is represented as a number for the compiler. This helps with readability.

#### Main code

    //============= Finite State Machine =============
    switch (currentState) {
        case StartScreen:
            Serial.println("Screen");
            checkConditions();
            if(buttonPressed || serialIn1){
            currentState = SelectionMenu; 
            buttonPressed = false;
            }
            break;

        case SelectionMenu:
            Serial.println("Menu");
            checkConditions();
            if(buttonPressed || serialIn0){
            currentState = StartScreen; //Go back
            buttonPressed = false;
            } 
            if(BP_pot1 || serialIn1){currentState = OsciMode;}
            if(BP_pot2 || serialIn2){currentState = LogicAnalyserMode;}
            if(BP_pot3 || serialIn3){currentState = FuncGenMode;}
            break;

        case OsciMode:
            Serial.println("OsciMode");
            checkConditions();
            if(buttonPressed || serialIn0){
            currentState = SelectionMenu;  //Go back
            buttonPressed = false;
            }
            break;

        case FuncGenMode:
            Serial.println("FuncGenMode");
            checkConditions();
            if(buttonPressed || serialIn0){
            currentState = SelectionMenu; //Go back
            buttonPressed = false;
            } 
            if(BP_pot1 || serialIn1){currentState = SquareWave;}
            if(BP_pot2 || serialIn2){currentState = SinWave;}
            if(BP_pot3){currentState = TriangleWave;}
            break;

        case LogicAnalyserMode:
            Serial.println("LogicAnalyserMode");
            checkConditions();
            if(buttonPressed || serialIn0){
            currentState = FuncGenMode;  //Go back
            buttonPressed = false;
            }
            break;

        case SquareWave:
            Serial.println("SquareWave");
            checkConditions();
            if(buttonPressed || serialIn0){
            currentState = FuncGenMode;  //Go back
            buttonPressed = false;
            }
            break;

        case SinWave:
            Serial.println("SinWave");
            checkConditions();
            if(buttonPressed || serialIn0){
            currentState = FuncGenMode;  //Go back
            buttonPressed = false;
            }
            break;

        case TriangleWave:
            Serial.println("TriangleWave");
            checkConditions();
            if(buttonPressed || serialIn0){
            currentState = FuncGenMode;  //Go back
            buttonPressed = false;
            }
            break;

        default:
            //ignore
            break;
        }

Each state includes a "checkConditions()" function for polling all the events.

### 3.3 OLED Screen

#### 3.3.1 I2C Protocol

I2C, or Inter-integrated circuit, is a synchronous, half-duplex communication protocol. This means that it includes a dedicate clock wire to synchronise the communication between devices, and a single data signal wire which can carry information in both directions, one direction at a time. Each device on using the I2c protocol needs an adress so that the master controller can select which device it wants to send or receive information from. A simplified diagram can be seen in figure 4 below. Pullup resistors are often added to discharge the capacitance induced charge between the two wires.

![I2C protocol](Project_Media/Images/I2c_protocol_basic.png)
*Figure 4: Simplified I2C protocol diagram  (Circuit Basics, n.d.)*

#### 3.3.2 Libraries

The SSD1306 required libraries to be utilised. The Adafruit_SSD1306 and Adafruit_GFX libraries were used for this.

#### 3.3.3 Finding OLED Screen's I2C Device Adress

An arduino example program was used to find the screens I2C device address. The program scans through standard 7-bit I2C adresses and outputs the device's address to the serial monitor if it finds it. It found that the SSD1306 OLED screen's device adress is "0x3C".

#### 3.3.4 Testing the OLED Screen

The OLED screen was tested using an example program from the Adafruit_SSD1306 library. The program scrolled through all its different drawing functions.

The first screen used had an issue with its reolution. The majority of the top of the screen appeared to have a reduced resolution. A new screen was given which worked properly.

### 3.4 Program Overview

#### 3.4.1 Minimal Oscilloscope

A function was created for a simple oscilloscope which took a reading from the "Ch1" analogue to digital converter (ADC) pin, displayed it on a set of axes on the screen, and printed the current voltage reading in millivolts on the screen. This function was used in FSM states; OsciMode, FuncGenMode, SquareWave, SinWave, and TriangleWave. This was first created as a stand-alone "sub-project" found [here](Sub_Projects/SimpleOscilloscope2) before being placed in a function.

The main code for the function can be seen below:

    //X Draw Values
    uint32_t prevXDrawVal = currXDrawVal-1;
    if(currXDrawVal >= 0 && currXDrawVal <= plotWidth){currXDrawVal++;} //Configure counter
    else{currXDrawVal = prevXDrawVal = 0; display.clearDisplay();}
    

    //Channel 1 A/D reading and mV mapping
    uint32_t potVal = analogRead(ch1Pin); 
    uint32_t volts_mV = map(potVal,0,1023,0,maxVoltage_mV);

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

This code works best for low-speed signals as it samples once per column for the screen. This way, it can only sample signals as fast as the screen can process which sub-optimal for speed.

#### 3.4.2 Function Generator

The function generator state in the program works as another menu screen.

## 4. Results

## 5. Discussion
