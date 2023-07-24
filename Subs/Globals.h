//
// <Globals.h>
//
// Global constant, #defines and  chars used in the code 
//
//
// These globals are similar to the Arduino code
// 
#ifndef MY_GLOBALS_H
#define MY_GLOBALS_H

const char Colon = ':';
const char exc = '!';
const char CR = '\r';
const char LF = '\n';
const char EndCh = LF;
const char EndLn = LF;

#define COMM_BUF_LIM    1024
#define KEY_BUF_LIM     80
#define MAX_TEXT_STRING 256

extern char KeyBuf[KEY_BUF_LIM];
extern char ErrorBuf[MAX_TEXT_STRING];
extern char TextBuf[MAX_TEXT_STRING];
extern const char* MsgSrc;

extern int ToScreen;
extern int AmServer;
extern const char* myTermOut1;


//
// the following 3 #defines are mutually exclusive and should be 
// selected by uncommenting the correct statements for the current build 
// and commenting out the other two

//#define MY_ARDUINO_BUILD
#define MY_LAPTOP_BUILD
//#define MY_RASPBERRY_PI_BUILD

#define MAX_CPU  8

#define IP_PORT 8080

#endif  // MY_GLOBALS_H


