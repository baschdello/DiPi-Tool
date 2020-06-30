#include <Arduino.h>

#include "menu.h"

//######################
//#      Strings       #
//######################
//misc strings
const char strBack[] PROGMEM = "Back";

//main-menu strings
const char strStart[] PROGMEM = "Start";
const char strSettings[] PROGMEM = "Settings";

//mode-menu strings
const char strTriggerControlled[] PROGMEM = "Trig controlled";
const char strTimerControlled[] PROGMEM = "Timer controlled";

//settings-menu strings
const char strTriggerSignal[] PROGMEM = "Trigger Signal";
const char strMode[] PROGMEM = "Mode";
const char strSave[] PROGMEM = "Save";


//######################
//#   Menu structure   #
//######################
//MENU_POINT structure:                   menu point string, previous menu point, next menu point, next horizontal menu point, menu specific function

//main-menu points
const MENU_POINT mpStart PROGMEM          = {strStart, TOP, SETTINGS, BOTTOM, fctDiPi};
const MENU_POINT mpSettings PROGMEM       = {strSettings, START, BOTTOM, TRIGGERSIGNAL, NULL};

//settings-menu points
const MENU_POINT mpTriggerSignal PROGMEM  = {strTriggerSignal, TOP, MODE, BOTTOM, fctTriggerSignal};
const MENU_POINT mpMode PROGMEM           = {strMode, TRIGGERSIGNAL, SAVE, BOTTOM, fctMode};
const MENU_POINT mpSave PROGMEM           = {strSave, MODE, SETTINGSBACK, BOTTOM, fctSave};
const MENU_POINT mpSettingsBack PROGMEM   = {strBack, SAVE, BOTTOM, SETTINGS, NULL};

//menu array
const MENU_POINT* menue[] PROGMEM = { &mpStart, &mpSettings, &mpTriggerSignal, &mpMode, &mpSave, &mpSettingsBack};

//display state
DISPLAY_STATE displayStatus;
