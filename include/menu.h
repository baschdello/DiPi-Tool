#ifndef MENU_H_
#define MENU_H_

#include<stdint.h>

//general menu constants
#define TOP         -1
#define BOTTOM      -2

//main-menu constants
#define START         0
#define SETTINGS      1

//settings-menu constants
#define TRIGGERSIGNAL 2
#define MODE          3
#define SAVE          4
#define SETTINGSBACK  5


//executable functions for special menupoints
extern uint8_t fctDiPi();
extern uint8_t fctTriggerSignal();
extern uint8_t fctMode();
extern uint8_t fctSave();

typedef struct {
  const char *text;         // text of menu point
  int8_t previous;          // previous menu point
  int8_t next;              // next menu point
  int8_t hor;               // horizontal menu point
  uint8_t ( *fp )( void );  // function to run
} MENU_POINT;

typedef struct {
  unsigned displayed:1;
  unsigned runFunction:1;
} DISPLAY_STATE;

#endif
