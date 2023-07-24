//
// <MyDataTypes.cpp
// Some datatype definitions and helper functions for comms classes
// See header for details

#include "MyDataTypes.h"


SHORTI makeShort( MYBYTE loByte, MYBYTE hiByte)
{
	SHORTI val;
    val = (SHORTI)hiByte;
	val = (val << 8);
	val+= loByte;
	return val;
}

MYWORD makeMYWord(MYBYTE loByte, MYBYTE hiByte)
{ 
// same as MAKEWORD(a,b) this will disregard signbits
	MYWORD val;
	val = (MYWORD)hiByte;
	val = (val  << 8);
	val+= loByte;
	return val;
}

MYDWORD makeDWord(MYWORD loWord, MYWORD hiWord)
{
	MYDWORD val;
	val = (MYDWORD)hiWord;
	val = (val << 16 );
	val += (MYDWORD)(loWord);
	return val;
}

/*
MYLONGWORD makeLongWord( MYDWORD loDWord, MYDWORD hiDWord)
{
	MYLONGWORD val;
	val = (MYLONGWORD)(hiDWord);
	val = (val << 32);
	val += (MYLONGWORD)(loDWord);
	return val;
}
*/



