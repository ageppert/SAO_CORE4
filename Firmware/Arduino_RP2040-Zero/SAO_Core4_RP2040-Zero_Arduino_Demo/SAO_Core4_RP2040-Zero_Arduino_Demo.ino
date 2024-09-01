/* SAO Core4 Demo with Arduino 
  Github repo for SAO Core4:  https://github.com/ageppert/SAO_CORE4
  Project page for SAO Core4: https://hackaday.io/project/197235-sao-core4-a-nibble-of-core-memory-with-i2c

  Dependencies. 
    This demo will work with a huge range of IDEs and hardware, but it was developed and tested with the following:
    Arduino IDE 2.3.2 
      Install "Arduino Mbed OS RP2040 Boards" v4.1.5 with Arduino Boards Manager
      The Wire Library is included.
    RP2040-Zero (miniature Pico/RP2040 dev board) https://www.waveshare.com/rp2040-zero.htm

  Learn more about the IO Expander and ways to methods to drive it:
    https://tronixstuff.com/2011/08/26/arduino-mcp23017-tutorial/ (just uses Wire I2C directly by controlling the registers)
    https://learn.adafruit.com/adafruit-mcp23017-i2c-gpio-expander/arduino (uses Adafruit MCP23017 library, not used in this demo)
    https://www.makerguides.com/using-gpio-expander-mcp23017-with-arduino/ (uses Adafruit MCP23017 library, not used in this demo)

  Hardware Connections
    The bottom SAO port (X1) is intended to provide full access to the Core4 Matrix, LEDs, and sense circuit reset.
    SAO I2C pins connect to the GPIO Expander chip (MCP23017) which controls the core matrix transistors and LEDs.
    SAO GPIO1 enables power to the core matrix drive transistors
    SAO GPIO2 sense a core flip (change of the magnetic field from CW to CCW or vice versa, in the ferrite core)
  
    SAO I2C Pins are connected to the GPIO expander MCP23017 SDA and SCL pins
      All MCP23017 pins are configured as output.

    Default IO accessed by SAO GPIO pins, directly
      SAO_GPIO1 = unused, available
      SAO_GPIO2 = INPUT Core Matrix Sense, default low, unsensed
*/

#define HARDWARE_VERSION_MAJOR  0
#define HARDWARE_VERSION_MINOR  1
#define HARDWARE_VERSION_PATCH  1
#define MCU_FAMILY              RP2040
  /***************************************** CORE4 HARDWARE VERSION TABLE *******************************************
  | VERSION |  DATE      | MCU     | DESCRIPTION                                                                    |
  -------------------------------------------------------------------------------------------------------------------
  |  0.1.0  | 2019-08-23 | RP2040  | First prototype, as built. Confusing row drive logic!
  |  0.1.1  | 2019-08-31 | RP2040  | Rework 0.1.0, isolate GPB0/1 for 7N/7P, route GPB6/7 to 9N/9P, connect GPA6 
  |         |            |         |   directly to CM_EN with JP25
  |  0.2.0  | 2019-00-xx | RP2040  | WIP - not yet release - based on 0.1.1, reduce overall size, simplify.
  |         |            |         |
  -----------------------------------------------------------------------------------------------------------------*/

#include <Wire.h>                           // RP204 Pico-Zero default I2C port is GPIO4 is SDA0, GPIO5 is SCL0
#define IO_EXPANDER_ADDRESS         0x27    // SAO Core4 default MCP23017 address is 0x27
#define IO_EXPANDER_REG_PORTA_DIR   0x00
#define IO_EXPANDER_REG_PORTB_DIR   0x01
#define IO_EXPANDER_REG_PORTA_DATA  0x12
#define IO_EXPANDER_REG_PORTB_DATA  0x13
static bool LEDArray[4] =  {0, 0, 0, 0};    // Default all four LEDs off.
#define IO_PB_LED_ARRAY_START_BIT      2
#define IO_PA_CORE_MATRIX_ENABLE_BIT   6
#define IO_PA_SENSE_RESET_BIT          7
uint8_t IOPortAInactive = 0b00101010;
/*                          ||||||||__GPA0 : COLUMN Core Matrix Drive Transistor CMDQ-1N, XB0, default low, inactive
                            |||||||___GPA1 : COLUMN Core Matrix Drive Transistor CMDQ-1P, XB0, default high, inactive
                            ||||||____GPA2 : COLUMN Core Matrix Drive Transistor CMDQ-2N, XB1, default low, inactive
                            |||||_____GPA3 : COLUMN Core Matrix Drive Transistor CMDQ-2P, XB1, default high, inactive
                            ||||______GPA4 : COLUMN Core Matrix Drive Transistor CMDQ-3N, XT0,1, default low, inactive
                            |||_______GPA5 : COLUMN Core Matrix Drive Transistor CMDQ-3P, XT0,1, default high, inactive
                            ||________GPA6 : Core Matrix Enable, default low, inactive
                            |_________GPA7 : Core Matrix Sense Reset, default low, inactive */
uint8_t IOPortBInactive = 0b10000010;
/*                          ||||||||__GPB0 : ROW Core Matrix Drive Transistor CMDQ-7N, YL0, default low, inactive
                            |||||||___GPB1 : ROW Core Matrix Drive Transistor CMDQ-7P, YL0, default high, inactive
                            ||||||____GPB2 : LED_1 / Pixel 0, default low, inactive
                            |||||_____GPB3 : LED_2 / Pixel 1, default low, inactive
                            ||||______GPB4 : LED_3 / Pixel 2, default low, inactive
                            |||_______GPB5 : LED_4 / Pixel 3, default low, inactive
                            ||________GPB6 : ROW Core Matrix Drive Transistor CMDQ-9N, YL1, default low, inactive
                            |_________GPB7 : ROW Core Matrix Drive Transistor CMDQ-9P, YL1, default high, inactive */
uint8_t IOPortA = IOPortAInactive;
uint8_t IOPortB = IOPortBInactive;
#define PIN_SAO_GPIO_1        26      // Set up and use for whatever you want.
#define PIN_SAO_GPIO_2_SENSE  27      // Configured as input for sensing the core flip.
bool CMSenseArray[4] = {0,0,0,0};     // Store the most recent sense signal status for each core.
// These arrays set the row and column transistors correctly to address a given pixel with current in the correct direction.
// Pixel is 0 to 3. Value is 0 or 1. The value of 1 is arbitrarily defined by upper left Pixel 0 core with both wires + in upper left.
static uint8_t CMDColPA[4][2] = {
//{0b00101010, 0b00101010}, default reference for each transistor to be inactive
  {0b00111000, 0b00001011}, 
  {0b00110010, 0b00001110},
  {0b00111000, 0b00001011}, // TODO: update this row
  {0b00110010, 0b00001110} // TODO: update this row
/*     ||||||      ||||||__GPA0 : CMDQ-1N, XB0
       ||||||      |||||___GPA1 : CMDQ-1P, XB0
       ||||||      ||||____GPA2 : CMDQ-2N, XB1
       ||||||      |||_____GPA3 : CMDQ-2P, XB1
       ||||||      ||______GPA4 : CMDQ-3N, XT0,1
       ||||||      |_______GPA5 : CMDQ-3P, XT0,1
       ||||||__GPA0 : CMDQ-1N, XB0
       |||||___GPA1 : CMDQ-1P, XB0
       ||||____GPA2 : CMDQ-2N, XB1
       |||_____GPA3 : CMDQ-2P, XB1
       ||______GPA4 : CMDQ-3N, XT0,1
       |_______GPA5 : CMDQ-3P, XT0,1
       ______      _______
         |             |______ to write a one
         |____________________ to write a zero */
};
static uint8_t CMDRowPB[4][2] {
//{0b10000010, 0b10000010}, default reference for each transistor to be inactive
  {0b00000011, 0b11000000},
  {0b00000011, 0b11000000},
  {0b11000000, 0b00000011},
  {0b11000000, 0b00000011}
/*   ||    ||    ||    ||__GPB0 : CMDQ-7N, YL0
     ||    ||    ||    |___GPB1 : CMDQ-7P, YL0
     ||    ||    ||________GPB6 : CMDQ-9N, YL1
     ||    ||    |_________GPB7 : CMDQ-9P, YL1
     ||    ||__GPB0 : CMDQ-7N, YL0
     ||    |___GPB1 : CMDQ-7P, YL0
     ||________GPB6 : CMDQ-9N, YL1
     |_________GPB7 : CMDQ-9P, YL1
     ________     ________
         |             |______ to write a one
         |____________________ to write a zero */
};
enum TopLevelMode                       // Top Level Mode State Machine
{
  MODE_INIT,
  MODE_DAZZLE,   
  MODE_FLUX_TEST,   
  MODE_GAME_OF_MEMORY,   
  MODE_AT_THE_END_OF_TIME
} ;
uint8_t  TopLevelMode = MODE_INIT;
uint8_t  TopLevelModeDefault = MODE_DAZZLE;
uint32_t ModeTimeoutDeltams = 0;
uint32_t ModeTimeoutFirstTimeRun = true;


void setup()
{
  // Nothing to see here. Everything runs in a simple state machine in the main loop function, and the bottom of this file.
}

bool SerialInit() {
  bool StatusSerial = 1;
  Serial.begin(115200);
  // TODO get rid of this delay, quickly detect presence of serial port, and send message only if it's detected.
  delay(1500);
  Serial.println("\nHello World! This is the serial port talking...");
  return StatusSerial;
}

void IOExpanderSafeStates() {
  Wire.beginTransmission(IO_EXPANDER_ADDRESS);
  Wire.write(IO_EXPANDER_REG_PORTA_DATA);
  Wire.write(IOPortAInactive);
  Wire.endTransmission();
  Wire.beginTransmission(IO_EXPANDER_ADDRESS);
  Wire.write(IO_EXPANDER_REG_PORTB_DATA);
  Wire.write(IOPortBInactive);
  Wire.endTransmission();
}

bool IOExpanderInit() {
  Serial.print(">>> INIT IO Expander started... ");
  bool StatusIOExpander = 1;                      // default 0 = I2C MCP23017 not found, or 1 = I2C MCP23017 is found
  // TODO: detect MCP23017 and set status flag
  Wire.begin();                                   // wake up I2C bus, one time only
  Serial.print(" set data port to safe/inactive state,");
  IOExpanderSafeStates();
  Serial.print(" set all ports as ouput ");
  Wire.beginTransmission(IO_EXPANDER_ADDRESS);
  Wire.write(IO_EXPANDER_REG_PORTA_DIR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(IO_EXPANDER_ADDRESS);
  Wire.write(IO_EXPANDER_REG_PORTB_DIR);
  Wire.write(0x00);
  Wire.endTransmission();
  Serial.print("...completed with status ");
  if (StatusIOExpander ) { Serial.println("PRESENT.");}
  else { Serial.println("NOT present.");}
  return StatusIOExpander;
}

void SAOSensePinInit (){
  pinMode(PIN_SAO_GPIO_2_SENSE, INPUT);
}

void ModeTimeOutCheckReset () {
  ModeTimeoutDeltams = 0;
  ModeTimeoutFirstTimeRun = true;
}

bool ModeTimeOutCheck (uint32_t ModeTimeoutLimitms) {
  static uint32_t NowTimems;
  static uint32_t StartTimems;
  NowTimems = millis();
  if(ModeTimeoutFirstTimeRun) { StartTimems = NowTimems; ModeTimeoutFirstTimeRun = false;}
  ModeTimeoutDeltams = NowTimems-StartTimems;
  if (ModeTimeoutDeltams >= ModeTimeoutLimitms) {
    ModeTimeOutCheckReset();
    Serial.print(">>> Mode timeout after ");
    Serial.print(ModeTimeoutLimitms);
    Serial.println(" ms.");
    return (true);
  }
  else {
    return (false);
  }
}

void LEDUpdate() {
  // Update the bitfield to be sent to the port
  for (uint8_t i = 0; i < 4; i++) {
    if (LEDArray[i])  { IOPortB |=  (1 << (IO_PB_LED_ARRAY_START_BIT+i)); }   // Set the bit at position to 1
    else              { IOPortB &= ~(1 << (IO_PB_LED_ARRAY_START_BIT+i)); }   // Clear the bit at position to 0
  }
  // Update the port
  Wire.beginTransmission(IO_EXPANDER_ADDRESS);
  Wire.write(IO_EXPANDER_REG_PORTB_DATA);        // GPIOB
  Wire.write(IOPortB);                          // port B bit field
  Wire.endTransmission();
}

void PortAUpdate (uint8_t data) {
  Wire.beginTransmission(IO_EXPANDER_ADDRESS);
  Wire.write(IO_EXPANDER_REG_PORTA_DATA);
  Wire.write(data);
  Wire.endTransmission();
}

void PortBUpdate (uint8_t data) {
  Wire.beginTransmission(IO_EXPANDER_ADDRESS);
  Wire.write(IO_EXPANDER_REG_PORTB_DATA);
  Wire.write(data);
  Wire.endTransmission();
}

void CoreMemoryBitWrite(uint8_t pixel, bool value) {
  // Set up row and column transistors
  IOPortA = CMDColPA[pixel][value];
  IOPortB = CMDRowPB[pixel][value];
  // Reset the sense latch (high)
  IOPortA |=  (1 << IO_PA_SENSE_RESET_BIT);   // Set the bit at position to 1
  // Send it
  PortAUpdate(IOPortA);
  PortBUpdate(IOPortB);
  // Clear the sense latch (low)
  IOPortA &= ~(1 << IO_PA_SENSE_RESET_BIT);   // Clear the bit at position to 0
  // Enable the Core Matrix
  IOPortA |=  (1 << IO_PA_CORE_MATRIX_ENABLE_BIT);   // Set the bit at position to 1
  // Send it
  PortAUpdate(IOPortA);
  // Wait a tiny bit, but mostly not needed because the core flip happens in just under 1 us.
  // delay(1);
  // Disable the Core Matrix
  IOPortA &= ~(1 << IO_PA_CORE_MATRIX_ENABLE_BIT);   // Clear the bit at position to 0
  // Send it
  PortAUpdate(IOPortA);
  // Return the row and column transistors to safe states... or just skip it to save time
  // Send it
  // Check for the sense signal and update the Core Matrix Sense array with the value
  CMSenseArray[pixel] = digitalRead(PIN_SAO_GPIO_2_SENSE);
}

void loop()
{
  switch(TopLevelMode) {
    case MODE_INIT: {
      SerialInit();
      Serial.println(">>> Entered MODE_INIT.");
      IOExpanderInit();
      SAOSensePinInit();
      Serial.println("");
      Serial.println("  |-------------------------------------------------------------------------| ");
      Serial.println("  | Welcome to the SAO Core4 Demo with Arduino IDE 2.3.2 using RP2040-Zero! | ");
      Serial.println("  |-------------------------------------------------------------------------| ");
      Serial.println("");
      ModeTimeOutCheckReset(); 
      TopLevelMode = MODE_DAZZLE;
      Serial.println(">>> Leaving MODE_INIT.");
      break;
    }
    case MODE_DAZZLE: {
      if (ModeTimeoutFirstTimeRun) { Serial.println(">>> Entered MODE_DAZZLE."); }
      static uint8_t PixelToTurnOn = 0;
      // Update the pixel to be turned on
      switch (PixelToTurnOn) {
        case 0: { PixelToTurnOn = 1; break; }
        case 1: { PixelToTurnOn = 3; break; }
        case 3: { PixelToTurnOn = 2; break; }
        case 2: { PixelToTurnOn = 0; break; }
        default: { PixelToTurnOn = 0; break; }
      }
      // Update the LED array
      for (uint8_t i = 0; i < 4; i++) {
        if (PixelToTurnOn == i) { LEDArray[i] = 1; }
        else                    { LEDArray[i] = 0; }
      }
      LEDUpdate();        
      delay(80);

      if (ModeTimeOutCheck(3000)){ 
        ModeTimeOutCheckReset();
        for (uint8_t i = 0; i < 4; i++) { LEDArray[i] = 0; }
        LEDUpdate();
        TopLevelMode = MODE_FLUX_TEST; 
        Serial.println(">>> Leaving MODE_DAZZLE.");
      }
      break;
    }
    case MODE_FLUX_TEST: {
      if (ModeTimeoutFirstTimeRun) { 
        Serial.println(">>> Entered MODE_FLUX_TEST."); 
        // ModeTimeOutCheckReset(); 
        ModeTimeoutFirstTimeRun = false;
        IOExpanderSafeStates();
        }
      // Write all core 0, and if they change state as expected, don't light up an LED.
      for (uint8_t i = 0; i < 4; i++) {
        CoreMemoryBitWrite(i,0);
        LEDArray[i] = !CMSenseArray[i];
      }
      LEDUpdate(); 
      delay(25);
      // Write all core 1, and if they change state as expected, don't light up an LED.
      for (uint8_t i = 0; i < 4; i++) {
        CoreMemoryBitWrite(i,1);
        LEDArray[i] = !CMSenseArray[i];
      }
      LEDUpdate(); 
      delay(25);

      // TODO: Check for SAO GPIO1 to go high and move to MODE_GAME_OF_MEMORY
      break;
    }

    case MODE_GAME_OF_MEMORY: {
      if (ModeTimeoutFirstTimeRun) { 
        Serial.println(">>> Entered MODE_GAME_OF_MEMORY."); 
        //ModeTimeOutCheckReset(); 
        ModeTimeoutFirstTimeRun = false; 
        IOExpanderSafeStates();
      }




      
      break;
    }

    case MODE_AT_THE_END_OF_TIME: {
      Serial.println(">>> Stuck in the MODE_AT_THE_END_OF_TIME! <<<");
      break;
    }

    default: {
      Serial.println(">>> Something is broken! <<<");
      break;
    }
  }
}
