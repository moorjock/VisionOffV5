//
// <MyDataTypes.h>
//
// These provide consistent sizeof() data types across different platforms
// Important when communicating data across comms links.
//
// However there are some inconsistencies in Arduino models
// The MEGA and UNO (AVR) boards use short int (16-bit) as the 
// base int type whilst the Due uses a 32 bit int
// char, float and double data types are the same IEEE standard 
// on all types so no need to be #defined here
//
// The following #defines and functions ensure byte consistency 
// in communications there is no need to proliferate them further
// also probably best avoid MYLONGWORD
//-----------------------------------------------------------------------------------
//  Version:     1.0
//  Changes:     First version
//  Date:        01 October 2019 
//  Programmer:  A. Shepherd
//-----------------------------------------------------------------------------------
//
#ifndef MYDATATYPES_H
#define MYDATATYPES_H

// The following three #defines are mutually exclusive 
// uncomment ONLY one of them for the target machine on which the code is to be compiled
// I dont like compiler directives and the code may get too complex with more than 
// 3 processors however compiling the code for the given machine is efficient
// Note: additional changes are needed in MyDataTypes.h for the Arduino

#ifdef MY_ARDUINO_BUILD
#define SHORTI     int
#define MYINT      long
#define MYBYTE     unsigned char
#define MYWORD     unsigned int
#define MYDWORD    unsigned long int
// #define MYLONGWORD unsigned long int

#else

#define SHORTI     short int
#define MYINT      int
#define MYBYTE     unsigned char
#define MYWORD     unsigned short int
#define MYDWORD    unsigned int
//#define MYLONGWORD unsigned long int
//
// Arduino clones
//
inline MYBYTE highByte(SHORTI val)
{
  return (MYBYTE)(val >> 8);
}
inline MYBYTE lowByte(SHORTI val)
{
  return (MYBYTE)(val);
}

#endif

SHORTI makeShort( MYBYTE loByte, MYBYTE hiByte);

MYWORD makeMYWord(MYBYTE loByte, MYBYTE hiByte);

MYDWORD makeDWord(MYWORD loWord, MYWORD hiWord);



#endif

