#include <Arduino.h>

#include "pin_definitions.h"
#include "menu.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define DEBOUNCE_TIME   5 //ms
#include <Bounce2.h>

//#define DEBUG_USB

#define ICON_ARROW_UP        0
#define ICON_ARROW_DOWN      1

#define PERMANENT   0
#define PULSE       1
#define TRIGGER     0
#define TIMER       1
#define MILISECONDS 0
#define SECONDS     1

typedef struct  {
    unsigned buttonDispense:1;
    unsigned buttonPickplace:1;
    unsigned buttonRotaryEnc:1;
    unsigned rotaryEncLeft:1;
    unsigned rotaryEncRight:1;
    unsigned stateChanged:1;
}t_buttonStates;

typedef struct {
  unsigned triggerSignal:1;
  unsigned mode:1;
  unsigned timerActive:1;
  unsigned time:6;
  unsigned timeReached:6;
}t_settings;

//Menu array
extern const MENU_POINT* const menue[] PROGMEM;

//Display state
extern DISPLAY_STATE displayStatus;

void initHardware();
void updateInput();
void displayMenu();
void runMenuSpecificFunction();
void writeValues(); //write settings to EEPROM
void readValues();  //read settings from EEPROM

//Helper functions for valves and LEDs
void valveDispense(uint8_t);
void valvePickPlace(uint8_t);
inline void ledsOff();

//Fancy icons for scroll bar
byte arrowUp[8] = {132,142,149,132,132,132,132,132};
byte arrowDown[8] = {132,132,132,132,132,149,142,132};

//Bounce objects for switches
Bounce bounceDispense   = Bounce(SWITCH_DISPENSE,DEBOUNCE_TIME);
Bounce bouncePickPlace  = Bounce(SWITCH_PICKPLACE,DEBOUNCE_TIME);
Bounce bounceRotarySw   = Bounce(SWITCH_ROTARY,DEBOUNCE_TIME);


LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

HardwareTimer timer(TIM1);
volatile int8_t enc_delta;

t_buttonStates input;
t_settings settings;

volatile uint8_t timeReached;
volatile uint8_t i;

uint8_t menuPunkt = START;  //first menu point after power on

void setup()
{
  #ifdef DEBUG_USB
    Serial.print(F(">Setup start\r\n" ));
  #endif

  // initialize the serial port:
  #ifdef DEBUG_USB
    Serial.begin(9600);
  #endif

  readValues();			//read settings from EEPROM

  initHardware();   //initialize display, buttons, encoder, timer...

  #ifdef DEBUG_USB
    Serial.print(F(">Setup end\r\n" ));
  #endif
}

void loop()
{
  updateInput();
  displayMenu();
  runMenuSpecificFunction();
}

inline void handler_rotaryEncoder()
{
  static int8_t last;
  int8_t neu, diff;

  neu = 0;
  if( digitalRead(ROTENC_A) )
    neu = 3;
  if( digitalRead(ROTENC_B) )
    neu ^= 1;           // convert gray to binary
  diff = last - neu;    // difference last - new
  if( diff & 1 )
  {                     // bit 0 = value (1)
    last = neu;         // store new as next last
    enc_delta += (diff & 2) - 1;    // bit 1 = direction (+/-)
  }
}

int8_t encode_read1( void )      // read single step encoders
{
  int8_t val;

  timer.pause();
  val = enc_delta;
  enc_delta = 0;
  timer.resume();
  return val;         // counts since last call
}

int8_t encode_read2( void )      // read two step encoders
{
  int8_t val;

  timer.pause();
  val = enc_delta;
  enc_delta &= 1;
  timer.resume();
  return val >> 1;
}


int8_t encode_read4( void )     // read four step encoders
{
  int8_t val;

  timer.pause();
  val = enc_delta;
  enc_delta &= 3;
  timer.resume();
  return val >> 2;
}

void millisTimerHandler(HardwareTimer *timer)
{
  if(settings.timerActive)
  {
    i++;
    if(i>=100)
    {
      i = 0;
      if(timeReached < settings.time-1)
        timeReached++;
      else
      {
        settings.timerActive = i = 0;
        valveDispense(LOW);
        valvePickPlace(LOW);
      }
    }
  }

  handler_rotaryEncoder();
}

void initHardware()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);


  //LCD
  lcd.begin(16, 2);
  lcd.createChar(ICON_ARROW_UP, arrowUp);
  lcd.createChar(ICON_ARROW_DOWN, arrowDown);
  lcd.clear();
  lcd.print("DiPi-Tool");
  lcd.setCursor(0,1);
  lcd.print("Software v1.0");
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);

  timer.pause();
  timer.setMode(1, TIMER_OUTPUT_COMPARE);
  timer.setCount(1*1000, MICROSEC_FORMAT); //1ms
  timer.attachInterrupt(1, millisTimerHandler);
  timer.refresh();
  timer.resume();

  #ifdef DEBUG_USB
    Serial.begin(9600);
  #endif

  //Switches
  pinMode(SWITCH_DISPENSE, INPUT_PULLUP);
  pinMode(SWITCH_PICKPLACE, INPUT_PULLUP);
  pinMode(SWITCH_ROTARY, INPUT_PULLUP);

  bounceDispense.attach(SWITCH_DISPENSE);
  bounceDispense.interval(5);
  bouncePickPlace.attach(SWITCH_PICKPLACE);
  bouncePickPlace.interval(5);
  bounceRotarySw.attach(SWITCH_ROTARY);
  bounceRotarySw.interval(5);

  //LEDs
  pinMode(LED_GREEN_DISPENSE, OUTPUT);
  pinMode(LED_GREEN_PICKPLACE, OUTPUT);
  pinMode(LED_RED_DISPENSE, OUTPUT);
  pinMode(LED_RED_PICKPLACE, OUTPUT);

  //RotaryEncoder
  pinMode(ROTENC_A, INPUT);
  pinMode(ROTENC_B, INPUT);

  //Valves
  pinMode(VALVE_DISPENSE,OUTPUT);
  pinMode(VALVE_PICKPLACE,OUTPUT);

}

void updateInput()
{
    input.rotaryEncLeft = input.rotaryEncRight = input.stateChanged = 0;  //reset state flags

    if(bounceDispense.update()) //button state changed
    {
        input.buttonDispense = bounceDispense.read() == 0? 1:0;
        input.stateChanged = 1;
    }
    if(bouncePickPlace.update()) //button state changed
    {
        input.buttonPickplace = bouncePickPlace.read() == 0? 1:0;
        input.stateChanged = 1;
    }
    if(bounceRotarySw.update()) //button state changed
    {
        input.buttonRotaryEnc = bounceRotarySw.read() == 0? 1:0;
        input.stateChanged = 1;
    }
    int rotaryEncoderReturn = encode_read2();
    if( rotaryEncoderReturn < 0)
    {
      input.rotaryEncRight = 1;
      input.stateChanged = 1;
    }
    else if( rotaryEncoderReturn > 0)
    {
      input.rotaryEncLeft = 1;
      input.stateChanged = 1;
    }

#ifdef DEBUG_USB
      if(input.stateChanged == 1)
      {
        Serial.print(F("switch state: "));
        Serial.print(input.buttonDispense,DEC);
        Serial.print(F(" "));
        Serial.print(input.buttonPickplace,DEC);
        Serial.print(F(" "));
        Serial.print(input.buttonRotaryEnc,DEC);
        Serial.print(F(" "));
        Serial.print(input.rotaryEncLeft,DEC);
        Serial.print(F(" "));
        Serial.print(input.rotaryEncRight,DEC);
        Serial.print(F("\r\n"));
      }
#endif
}

void displayMenu()
{
  if(input.stateChanged)
  {
    if(!displayStatus.runFunction)  //no function to run
    {                               //use rotary encoder for scrolling menu
      if(input.rotaryEncLeft && ((*(menue[menuPunkt])).previous != TOP))
      {
        menuPunkt = (*(menue[menuPunkt])).previous;
        displayStatus.displayed = 0;
      }
      if(input.rotaryEncRight && ((*(menue[menuPunkt])).next != BOTTOM))
      {
        menuPunkt = (*(menue[menuPunkt])).next;
        displayStatus.displayed = 0;
      }
      if(input.buttonRotaryEnc)
      {
        if( ((*(menue[menuPunkt])).fp == NULL) && ((*(menue[menuPunkt])).hor != BOTTOM) )
        {
          menuPunkt = ((*(menue[menuPunkt])).hor);
          displayStatus.displayed = 0;
        }
        else if( ((*(menue[menuPunkt])).fp != NULL) && ((*(menue[menuPunkt])).hor == BOTTOM) )
        {
          displayStatus.runFunction = 1;
		      input.buttonRotaryEnc = 0;	//reset
          displayStatus.displayed = 0;
        }
      }
    }
  }

  if( (!(displayStatus.displayed)) && (!(displayStatus.runFunction)) )
  {
    lcd.clear();
    if((*(menue[menuPunkt])).previous != TOP) //show scroll-up icon
    {
      lcd.setCursor(15,0);
      lcd.write(ICON_ARROW_UP);
    }
    if((*(menue[menuPunkt])).next != BOTTOM) //show scroll-down icon
    {
      lcd.setCursor(15,1);
      lcd.write(ICON_ARROW_DOWN);
    }
    lcd.setCursor(0,0);
    lcd.print(">");
    lcd.setCursor(1,0);     //set curser position to line 1, char 2
    lcd.print((*(menue[menuPunkt])).text);
    if((*(menue[menuPunkt])).next != BOTTOM)
    {
      lcd.setCursor(1,1);  //set curser position to line 2, char 2
      lcd.print((*(menue[(*(menue[menuPunkt])).next])).text);
    }

    displayStatus.displayed = 1;
  }
}

void runMenuSpecificFunction()
{
  uint8_t exitFkt = 0;
  if(displayStatus.runFunction)
  {
    exitFkt = ((*((*(menue[menuPunkt])).fp))());  //run menu specific function
  	if(exitFkt == 1)
  	{
  		displayStatus.runFunction = 0;
  		displayStatus.displayed = 0;
  	}
  }
}

void writeValues()
{
  EEPROM.write(0, settings.triggerSignal);
  EEPROM.write(1, settings.mode);
  EEPROM.write(2, settings.time);
}

void readValues()
{
  settings.triggerSignal = EEPROM.read(0);
  settings.mode = EEPROM.read(1);
  settings.time = EEPROM.read(2);
}

inline uint8_t buttonDispensePressed()
{
  return input.buttonDispense;
}
inline uint8_t buttonPickplacePressed()
{
  return input.buttonPickplace;
}
inline uint8_t buttonRotaryEncPressed()
{
  return input.buttonRotaryEnc;
}
inline uint8_t rotaryEncLeftTurned()
{
  return input.rotaryEncLeft;
}
inline uint8_t rotaryEncRightTurned()
{
  return input.rotaryEncRight;
}


uint8_t fctDiPi()
{
  static uint8_t valueChanged;
  char timBuf[5];
  if( !(displayStatus.displayed) )
  {
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("DiPi");
    lcd.setCursor(0,2);
    if(settings.mode == TRIGGER)
      lcd.print(" Trigger");
    else if(settings.mode == TIMER)
    {
      lcd.print(">Timer:      ms ");
      lcd.setCursor(8,1);
      sprintf(timBuf, "%4d", settings.time*100);
      lcd.print(timBuf);
    }
    digitalWrite(LED_GREEN_DISPENSE,HIGH);
    digitalWrite(LED_GREEN_PICKPLACE,HIGH);
    displayStatus.displayed = 1;
  }
  if( input.stateChanged)
  {
    if(settings.mode == TIMER && (rotaryEncRightTurned() || rotaryEncLeftTurned()))
    {
      //Encoder
      if( rotaryEncRightTurned() && settings.time < 63)
      {
        settings.time += 1;
        valueChanged = 1;
      }
      else if( rotaryEncLeftTurned() && settings.time > 1)
      {
        settings.time -= 1;
        valueChanged = 1;
      }
      if(valueChanged)
      {
        lcd.setCursor(8,1);
        lcd.print("     ms ");
        lcd.setCursor(8,1);
        sprintf(timBuf, "%4d", settings.time*100);
        lcd.print(timBuf);
        valueChanged = 0;
      }
    }
    //Switch 1
    if(buttonDispensePressed())
    {
      if(settings.mode == TIMER)
      {
        valveDispense(HIGH);
        valvePickPlace(LOW);
        timeReached=i=0;  //initialize timer vars
        settings.timerActive=1; //start timer
      }
      else if(settings.triggerSignal == PULSE && digitalRead(VALVE_DISPENSE))
        valveDispense(LOW);
      else
      {
        valveDispense(HIGH);
        valvePickPlace(LOW);
      }
    }
    else if(settings.triggerSignal == PERMANENT)
    {
      if(settings.mode != TIMER && digitalRead(VALVE_DISPENSE))
        valveDispense(LOW);
    }
    //Switch 2
    if(buttonPickplacePressed())
    {
      if(settings.mode == TIMER)
      {
        valveDispense(LOW);
        valvePickPlace(HIGH);
        timeReached=i=0;  //initialize timer vars
        settings.timerActive=1; //start timer
      }
      else if(settings.triggerSignal == PULSE && digitalRead(VALVE_PICKPLACE))
        valvePickPlace(LOW);
      else
      {
        valveDispense(LOW);
        valvePickPlace(HIGH);
      }
    }
    else if(settings.triggerSignal == PERMANENT)
    {
      if(settings.mode != TIMER)
      {
        valvePickPlace(LOW);
      }
    }

    //Switch 3
    if( buttonRotaryEncPressed() )
    {
      ledsOff();
      return 1;   //exit function and return to menu
    }
  }
  return 0;       //run menu one more time
}
uint8_t fctTriggerSignal()
{
  static uint8_t valueChanged = 0;
  if( !(displayStatus.displayed) )
  {
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("Trigger Signal");
    lcd.setCursor(0,2);
    if(settings.triggerSignal == PERMANENT)
      lcd.print(">permanent");
    else if(settings.triggerSignal == PULSE)
      lcd.print(">pulse");
    displayStatus.displayed = 1;
  }
  if( input.stateChanged)
  {
    if(rotaryEncRightTurned() || rotaryEncLeftTurned())
    {
      if( rotaryEncRightTurned() && settings.triggerSignal != PERMANENT)
      {
        settings.triggerSignal = PERMANENT;
        valueChanged = 1;
      }
      else if( rotaryEncLeftTurned() && settings.triggerSignal != PULSE)
      {
        settings.triggerSignal = PULSE;
        valueChanged = 1;
      }
      if(valueChanged)
      {
        lcd.setCursor(1,1);
        lcd.print("               ");
        lcd.setCursor(1,1);
        if(settings.triggerSignal == PERMANENT)
          lcd.print("permanent");
        else if(settings.triggerSignal == PULSE)
          lcd.print("pulse");
        valueChanged = 0;
      }
    }
  }
  if( buttonRotaryEncPressed() )
  {
    return 1;   //exit function and return to menu
  }
  return 0;     //run menu one more time
}
uint8_t fctMode()
{
  static uint8_t valueChanged = 0;
  if( !(displayStatus.displayed) )
  {
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("Mode");
    lcd.setCursor(0,1);
    if(settings.mode == TRIGGER)
      lcd.print(">Trigger mode");
    else if(settings.mode == TIMER)
      lcd.print(">Timer mode");
    displayStatus.displayed = 1;
  }
  if( input.stateChanged)
  {
    if(rotaryEncRightTurned() || rotaryEncLeftTurned())
    {
      if( rotaryEncRightTurned() && settings.mode != TRIGGER)
      {
        settings.mode = TRIGGER;
        valueChanged = 1;
      }
      else if( rotaryEncLeftTurned() && settings.mode != TIMER)
      {
        settings.mode = TIMER;
        valueChanged = 1;
      }
      if(valueChanged)
      {
        lcd.setCursor(1,1);
        lcd.print("               ");
        lcd.setCursor(1,1);
        if(settings.mode == TRIGGER)
          lcd.print("Trigger mode");
        else if(settings.mode == TIMER)
          lcd.print("Timer mode");
        valueChanged = 0;
      }
    }
  }
  if( buttonRotaryEncPressed() )
  {
    return 1;   //exit function and return to menu
  }
  return 0;     //run menu one more time
}

uint8_t fctSave()
{
  if( !(displayStatus.displayed) )
  {
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("Save");
    lcd.setCursor(0,1);
    lcd.print(">Please wait");
    lcd.setCursor(1,1);
    writeValues();
    displayStatus.displayed = 1;
  }
  lcd.setCursor(1,1);
  lcd.print("Settings saved!");

  if( buttonRotaryEncPressed() )
  {
	  return 1;   //exit function and return to menu
  }
  return 0;     //run menu one more time
}

void valveDispense(uint8_t i)
{
  if(i==LOW)
  {
    digitalWrite(VALVE_DISPENSE,LOW);

    digitalWrite(LED_RED_DISPENSE,LOW);

    digitalWrite(LED_GREEN_DISPENSE,HIGH);
  }
  else
  {
    digitalWrite(VALVE_DISPENSE,HIGH);

    digitalWrite(LED_RED_DISPENSE,HIGH);

    digitalWrite(LED_GREEN_DISPENSE,LOW);
  }
}
void valvePickPlace(uint8_t i)
{
  if(i==LOW)
  {
    digitalWrite(VALVE_PICKPLACE,LOW);

    digitalWrite(LED_RED_PICKPLACE,LOW);

    digitalWrite(LED_GREEN_PICKPLACE,HIGH);
  }
  else
  {
    digitalWrite(VALVE_PICKPLACE,HIGH);

    digitalWrite(LED_RED_PICKPLACE,HIGH);

    digitalWrite(LED_GREEN_PICKPLACE,LOW);
  }
}

inline void ledsOff()
{
  digitalWrite(LED_GREEN_DISPENSE,LOW);
  digitalWrite(LED_GREEN_PICKPLACE,LOW);
  digitalWrite(LED_RED_DISPENSE,LOW);
  digitalWrite(LED_RED_PICKPLACE,LOW);
}
