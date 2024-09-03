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
#define PIN_SAO_GPIO_1_MODE   26      // Configured as input for mode switching or general user use.
#define PIN_SAO_GPIO_2_SENSE  27      // Configured as input for sensing the core flip.
bool CMSenseArray[4] = {0,0,0,0};     // Store the most recent sense signal status for each core.
// These arrays set the row and column transistors correctly to address a given pixel with current in the correct direction.
// Pixel is 0 to 3. Value is 0 or 1. The value of 1 is arbitrarily defined by upper left Pixel 0 core with both wires + in upper left.
// CMDColPA = Core Matrix Drive Columns on Port A
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

void SAOGPIOPinInit (){
  pinMode(PIN_SAO_GPIO_2_SENSE, INPUT);
  pinMode(PIN_SAO_GPIO_1_MODE, INPUT_PULLUP);
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

void LEDClear() {
  for (uint8_t j = 0; j < 4; j++) { LEDArray[j] = 0; }
}

void LEDFill() {
  for (uint8_t j = 0; j < 4; j++) { LEDArray[j] = 1; }
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
  // Copy in the LED array status in Port to avoid just shutting them off.
  for (uint8_t j = 0; j < 4; j++) { 
    if (LEDArray[j]) {
      IOPortB |=  (1 << (IO_PB_LED_ARRAY_START_BIT+j));   // Set the bit at position to 1
    }
    else {
      IOPortB &= ~(1 << (IO_PB_LED_ARRAY_START_BIT+j));  // Clear the bit at position to 0
    }
  }
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
      SAOGPIOPinInit();
      Serial.println("");
      Serial.println("  |-------------------------------------------------------------------------| ");
      Serial.println("  | Welcome to the SAO Core4 Demo with Arduino IDE 2.3.2 using RP2040-Zero! | ");
      Serial.println("  |-------------------------------------------------------------------------| ");
      Serial.println("");
      ModeTimeOutCheckReset(); 
      TopLevelMode = TopLevelModeDefault;
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
        LEDClear();
        LEDUpdate();
        TopLevelMode++; 
        Serial.println(">>> Leaving MODE_DAZZLE.");
      }
      break;
    }

    case MODE_FLUX_TEST: {
      if (ModeTimeoutFirstTimeRun) { 
        Serial.println(">>> Entered MODE_FLUX_TEST."); 
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

      if (ModeTimeOutCheck(15000)){ 
        ModeTimeOutCheckReset();
        for (uint8_t i = 0; i < 4; i++) { LEDArray[i] = 0; }
        LEDUpdate();
        TopLevelMode = MODE_GAME_OF_MEMORY; 
        Serial.println(">>> Leaving MODE_FLUX_TEST.");
      }
      
      // Check for SAO GPIO1 to go low and move to another mode.
      if ( !digitalRead(PIN_SAO_GPIO_1_MODE) ){
        Serial.println("Mode changed!");
        TopLevelMode = MODE_GAME_OF_MEMORY;
      }
      break;
    }

    case MODE_GAME_OF_MEMORY: {
      #define GAME_MEMORY_SEQUENCE_MAX_LENGTH 255
      static uint8_t GameMemoryState = 0;                 // Game state machine
      static uint8_t GameMemorySequenceLength = GAME_MEMORY_SEQUENCE_MAX_LENGTH-1;        // Maximum length of the memory sequence (array starts at 0, so that counts. If this line is 2, then there are actually 3 positions to memorize initially)
      static uint8_t GameMemoryTestStepPosition;      // Progress point in the sequence, where test is currently being evaluated
      static uint8_t GameMemoryTestStepEnd;           // How many steps in the sequence have been completed successfully
      static bool    GameMemoryTestStepComplete;  // A step in the sequence has been matched successfully
      static bool    GameMemoryPartialComplete;   // A partial sequence has been matched successfully
      static bool    GameMemoryTestStepWrong;
      static bool    GameMemoryGameComplete;      // The whole game sequence has been matched successfully
      static uint8_t GameMemorySequenceArray[GAME_MEMORY_SEQUENCE_MAX_LENGTH];
      static uint8_t PixelToTurnOn = 0;
      static bool      CoreMemoryIsTouched   = false;       // Register a touch
      static bool      CoreMemoryWasTouched   = false;      // Flag to catch state transition
      static uint8_t   CoreMemoryWhichIsTouched = 0;        // Keep track of which core is or was being touched
      static uint8_t   CoreMemoryCountTouched = 0;          // Keep track of how many cores are being touched
      static bool      CoreMemoryIsReleased  = true;        // Register a release
      static bool      CoreMemoryJustReleased = false;      // Register JUST released for one time use
      static uint16_t  CoreMemoryIsReleasedDebounce = 0;    // Keep track of how many times through this state the core is not touched.
      static uint16_t  CoreMemoryIsReleasedThresh = 3;     // How many times through the loop to register cores as not touched.
      // static bool      GameOKtoProceedToSeqStep   = false;  // Flag to indicate the correct core was touched in the sequence and move forward.
      
      if (ModeTimeoutFirstTimeRun) { 
        Serial.println(">>> Entered MODE_GAME_OF_MEMORY."); 
        ModeTimeoutFirstTimeRun = false; 
        IOExpanderSafeStates();
        LEDClear();
        LEDUpdate();        
        GameMemoryState = 0;
      }


      switch (GameMemoryState) {
        case 0: {                                               // Clear variables and game start-up sparkle LEDs.
          Serial.print(">>> MODE_GAME_OF_MEMORY state: ");
          Serial.println(GameMemoryState); 
          GameMemoryTestStepPosition = 0;
          GameMemoryTestStepEnd = 0;
          GameMemoryTestStepComplete = false;    
          GameMemoryPartialComplete = false; 
          GameMemoryGameComplete = false;    
          CoreMemoryIsTouched   = false;   
          CoreMemoryWasTouched   = false;  
          CoreMemoryWhichIsTouched;    
          CoreMemoryCountTouched = 0;      
          CoreMemoryIsReleased  = true;    
          CoreMemoryJustReleased = false;  
          CoreMemoryIsReleasedDebounce = 0;
          GameMemoryTestStepWrong = false;
          // Twinkle the LEDs (moved down into the randomizer)
/*         for (uint8_t i = 0; i <= 50; i++) {
            PixelToTurnOn = random(0, 4);
            // Update the LED array
            for (uint8_t j = 0; j < 4; j++) {
              if (PixelToTurnOn == j) { LEDArray[j] = 1; }
              else                    { LEDArray[j] = 0; }
            }
            LEDUpdate();        
            delay(20);
            LEDClear();
            LEDUpdate();
            delay(10);
          }
          LEDClear();
          LEDUpdate();
          delay(500);
*/
          GameMemoryState = 1;
          break;
        }

        case 1: {                                               // Generate all of the pixel positions for memory test.
          Serial.print(">>> MODE_GAME_OF_MEMORY state: "); 
          Serial.println(GameMemoryState);
          LEDClear();
          LEDUpdate();
          for (uint8_t i = 0; i < GameMemorySequenceLength; i++) {
            GameMemorySequenceArray[i] = random(0, 4);
            // Update the LED array
            for (uint8_t j = 0; j < 4; j++) {
              if (GameMemorySequenceArray[i] == j) { LEDArray[j] = 1; }
              else                    { LEDArray[j] = 0; }
            }
            LEDUpdate();      
            delay(20);
            LEDClear();
            LEDUpdate();
            delay(2);
          }
          LEDClear();
          LEDUpdate();
          delay(500);
          GameMemoryState = 2;
          break; 
        }

        case 2: {                                               // Show the pattern to memorize
          Serial.print(">>> MODE_GAME_OF_MEMORY state: "); 
          Serial.println(GameMemoryState); 
          Serial.print("    GameMemoryTestStepEnd "); 
          Serial.print(GameMemoryTestStepEnd+1); 
          Serial.print(" of "); 
          Serial.print(GameMemorySequenceLength+1);
          Serial.println(" possible."); 
          Serial.print("    Pattern to match is: "); 
          for (uint8_t i = 0; i <= GameMemoryTestStepEnd; i++) {
            Serial.print(GameMemorySequenceArray[i]);
            for (uint8_t j = 0; j < 4; j++) {
              if (j == GameMemorySequenceArray[i]) { LEDArray[j] = 1; }
              else                    { LEDArray[j] = 0; }
            }
            LEDUpdate();
            delay(750);
            LEDClear();
            LEDUpdate();
            delay(250);
          }
          Serial.println(); 
          LEDClear();
          LEDUpdate();       
          GameMemoryState = 3;
          break; 
          }

        case 3: {                                               // Wait, record touches, check for match up to GameMemoryPoints
          // Serial.print(">>> MODE_GAME_OF_MEMORY state: "); 
          // Serial.println(GameMemoryState);

          // Scan the core matrix for a touch and show it
          CoreMemoryCountTouched = 0;                           // Reset cores touched count to 0
          for (uint8_t i = 0; i < 4; i++) {
            CoreMemoryBitWrite(i,1);                            // Write all cores 1, 
            CoreMemoryBitWrite(i,0);                            // write all cores back to 0.
            LEDArray[i] = !CMSenseArray[i];                     // If core changed state as expected (CMSenseArray), set LEDArray position OFF.
            if (LEDArray[i]) {                                  // If the pixel has a magnet touching it,
              CoreMemoryWhichIsTouched = i;                     // keep track of the most recent and highest position is touched,
              CoreMemoryCountTouched++;                         // update touch count.
              Serial.print("    Pixel ");
              Serial.print(i); 
              Serial.print(" touched,");
            }
          }
          LEDUpdate();

          // Keep track of whether or not a core is touched, and debounce for a release, and flag "just released."
          if (CoreMemoryCountTouched) {                         // If any cores are touched,
            CoreMemoryIsTouched = true;                         // lock in touch flags
            CoreMemoryWasTouched = true;
            CoreMemoryIsReleased = false;
          }
          else {
            CoreMemoryIsTouched = false;
            CoreMemoryIsReleased = true;
          }
          if ( (CoreMemoryWasTouched) && (CoreMemoryIsReleased) ) {
            CoreMemoryIsReleasedDebounce++;                     // increment the debounce counter 
          }
          else {
            CoreMemoryIsReleasedDebounce = 0;                     // reset the debounce counter 
          }
          if (CoreMemoryIsReleasedDebounce > CoreMemoryIsReleasedThresh) {            // to make sure there really is no touch occurring.
            CoreMemoryIsReleasedDebounce = 0;                 // Reset the debounce counter.
            CoreMemoryWasTouched = false;
            CoreMemoryJustReleased = true;                    // One time use below!
          }

          // If core was touched, and just released, was it the correct one?
          if (CoreMemoryJustReleased) {
            if (CoreMemoryWhichIsTouched == GameMemorySequenceArray[GameMemoryTestStepPosition] ) {  // Correct pixel for this step of the sequence!
              Serial.print("GameMemoryTestStepPosition ");
              Serial.print(GameMemoryTestStepPosition); 
              Serial.print(" is correct with ");
              Serial.println(CoreMemoryWhichIsTouched);
              GameMemoryTestStepComplete = true;
            }
            else {                                                                                  // Wrong pixel touched!
              Serial.print("GameMemoryTestStepPosition ");
              Serial.print(GameMemoryTestStepPosition); 
              Serial.print(" is wrong with ");
              Serial.println(CoreMemoryWhichIsTouched);
              GameMemoryTestStepComplete = false;
              GameMemoryTestStepWrong = true;
            }
            CoreMemoryJustReleased = false;
          }

          if (GameMemoryTestStepComplete) {
            GameMemoryTestStepPosition++;                                                        // Get ready to test the next pixel.
            GameMemoryTestStepComplete = false;
          }

          if (GameMemoryTestStepPosition > GameMemoryTestStepEnd) {                              // Are we to the end of this sequence?
            GameMemoryTestStepEnd++;                                                             // Expand the test sequence.
            GameMemoryTestStepPosition = 0;                                                      // Reset test step position back to 0
            LEDClear();
            LEDUpdate();        
            delay(750);
            GameMemoryState = 2;                                                                 // Jump back to show the new extended sequence.
          }
          else {
            GameMemoryState = 3;                                                                 // Otherwise, wait for the next input in this sequence
          }
          if (GameMemoryTestStepEnd > GameMemorySequenceLength) {                          // User got to the end of the whole sequence!
            Serial.print(">>> YOU BEAT THE GAME OF MEMORY !!! "); 
            if (GameMemorySequenceLength < 255){
              GameMemorySequenceLength++;                                                         // Make the next sequence longer
            }
            GameMemoryState = 0;
          }

          if (GameMemoryTestStepWrong) {
              GameMemoryState = 4;
          }

          break; 
          }

        case 4: {                                               // Wrong sequence! Fill screen and begin the game again.
          Serial.print(">>> MODE_GAME_OF_MEMORY state: "); 
          Serial.println(GameMemoryState); 
          for (uint8_t i = 0; i < 3; i++) {
            LEDFill();
            LEDUpdate();
            delay(1000);
            LEDClear();
            LEDUpdate();        
            delay(500);
          }
          GameMemoryState = 0;
          break; 
          }
        
        default: { 
          Serial.println(">>> Invalid state in MODE_GAME_OF_MEMORY."); 
          break; 
          }
      }

      // Check for SAO GPIO1 to go low and move to another mode.
      if ( !digitalRead(PIN_SAO_GPIO_1_MODE) ) {
        Serial.println("Mode changed!");
        TopLevelMode = MODE_GAME_OF_MEMORY;
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
