//
// <CommTypes.cpp>
// Laptop Specific Version see header for details
//

#include "MyDataTypes.h"

#ifdef MY_ARDUINO_BUILD
#include <Arduino.h>
#include <stdlib>		// malloc, system, random atof, atoi etc
#include <new>			// C++ new & delete
#else
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>		// malloc, system, random atof, atoi etc
#include <memory.h>		// memcpy
#include <new>			// C++ new & delete
#include <errno.h>

#include "FMat.h"

#include "MyUnixSerial.h"
#include "MyUnixSocket.h"
#endif


#include "FMat.h"
#include "CommTypes.h"

CommType CommIn;

Command  CmdIn;
Command  CmdOut;
CommString TextOut;
CommString TextIn;


//
// these are general buffers and can be used anywhere
//
char KeyBuf[KEY_BUF_LIM];
char ErrorBuf[MAX_TEXT_STRING+KEY_BUF_LIM];
char TextBuf[MAX_TEXT_STRING];
char PrintBuf[MAX_TEXT_STRING];
char SeqBuf[MAX_TEXT_STRING];
char MesgBuf[MAX_TEXT_STRING];

//
// These classes are designed to communicate real-time data between processes
// to save memory allocation they are designed with frame objects in mind
// these are continually re-used rather than dynamically creating objects 
// for each message. They are not thread safe.
//
CommType::CommType()
{
    m_MyFD = -1;
    m_MyType=0;
    m_MyOpt = 0;

#ifndef MY_ARDUINO_BUILD
    m_pMyPort=NULL;     // note the port must be set before any comms can commence
#endif
}

int CommType::SetHead(int type, int opt)
{
    return BuildHead(type,opt);
}

int CommType::SetHead(CommType& Base)
{
//
// Effectively a Copy Constructor
// from an already valid Base Object (no checks)
//
    int i;
    m_MyType = Base.GetType();
    m_MyOpt = Base.GetOpt();

    for(i=0;i<6;i++){
      m_Head[i] = Base.m_Head[i];
    }

    return 0;
}

int CommType::BuildHead(int Type, int Opt)
{
//
// Used by all CommTypes to 
// Check validity and package up the message header as printable text
// This is to aid debugging and allow an ascii terminal to interrogate and
// debug the code either side of the comms link 
//
//  Type      1 = C = command
//            2 = T = Text message
//            3 = E = error message
//            4 = D = formated data packet
//                    Other types to follow
// int Opt    Depends on message 
//            if m_MyType = 1 = C m_MyOpt = the command number
//            if m_MyType = 2 = T m_MyOpt = length of the text message
//            if m_MyType = 3 = E m_MyOpt = the error number 
//            if m_MyType = 4 = D m_MyOpt = the type of data packet
//
    static char pad = ' ';
    int n1, n2, n3;
    static char nums[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
  //
    if(Type<1 || Type >4){
        return -1;
    }
    m_MyType = (SHORTI)Type;
    m_MyOpt = (SHORTI)Opt;
    m_Head[0] = ':';
    switch(Type){
        case 1:
            m_Head[1] = 'C';
            break;
        case 2:
            m_Head[1] = 'T';
            break;
        case 3:
            m_Head[1] = 'E';
            break;
        case 4:
            m_Head[1] = 'D';
            break;
        default:
            break;
    }

    if (Opt < 1 || Opt > 997) {
        return -2;  // the message is not valid type or length
    }

    if (Opt < 10) {
        m_Head[2] = pad;
        m_Head[3] = pad;
        m_Head[4] = nums[Opt];
    }
    else if ( Opt < 100 ) {
        n1 = Opt/10;
        n2 = Opt - 10*n1;
        m_Head[2] = pad;
        m_Head[3] = nums[n1];
        m_Head[4] = nums[n2];
    }
    else {
        n1 = Opt/100;
        n2 = (Opt - n1*100)/10;
        n3 = (Opt - n1*100 - n2*10);
        m_Head[2] = nums[n1];
        m_Head[3] = nums[n2];
        m_Head[4] = nums[n3];

    }
    m_Head[5] = '!';
    return 1;
}

int CommType::DecodeHeader(const char* pStr)
{
//
// Decodes a header from an incomming string 
//
// returns the integer type code if sucessful and 0 if not.
//
// This function checks that pStr contains a valid formatted header.
// For all message types these are the first 6 characters in the format :Annn!
//
// Type is returned if valid and the rest of the message can be built from 
// the remaining data on the read buffer
// There should be no more than 6 characters in pStr
//
    char Type = pStr[1];
    int i, len;
    char sub[3];

    len = strlen(pStr);
    if(len<6) return 0;

// tedious but robust
    if( (pStr[0]==':') && (pStr[5]=='!') ){
        switch(Type)
        {
            case 'C':
                m_MyType = MESG_COMMAND;
                break;
            case 'T':
                m_MyType = MESG_TEXT;
                break;
            case 'E':
                m_MyType = MESG_ERROR;
                break;
            case 'D':
                m_MyType = MESG_DATA;
                break;
            default:
                m_MyType = MESG_UNKNOWN;   // not found or not yet set
                return 0;
        }
        for (i = 0; i < 3; i++) {
            sub[i] = pStr[i + 2];
        }
        m_MyOpt = atoi(sub);
        return m_MyType;
    }
    return 0;
}
//
//---------------------------------------------------------------------------------------------
//
Command::Command(int Command)
{
    if(Command>0){
       BuildHead(MESG_COMMAND, Command);
    }
    else{
        m_MyType = MESG_COMMAND;
        m_MyOpt = 0;
    }
}


Command::Command(CommType& Base)
{
    SetCommand(Base);   // copy constructor
}

int Command::SetCommand(CommType& Base)
{
// Build the header for this command from the Base
// this is a copy operation with a validity check for the header
//
    int ret;
    if(Base.GetType()==MESG_COMMAND){
        ret = BuildHead(MESG_COMMAND,Base.GetOpt());
        return ret;
    }
    return 0;
}

// some common commands
void Command::InitDevice()
{
    BuildHead(MESG_COMMAND,RUN_INIT);
    Send();
}

void Command::Start()
{
    BuildHead(MESG_COMMAND,RUN_START);
    Send();
}
void Command::Stop()
{
    BuildHead(MESG_COMMAND,RUN_STOP);
    Send();
}

void Command::Reset()
{
    BuildHead(MESG_COMMAND,RUN_RESET);
    Send();
}
void Command::Version()
{
    BuildHead(MESG_COMMAND,RUN_VERSION);
    Send();
}
// the actual port will change from one device to another
// so this code will contain some couplings in Send() and Receive();
// it is also possible that a port will not be open depending on the
// configuration setup so only send if m_MyFD is set
//
int Command::Send()
{
#ifdef MY_ARDUINO_BUILD
    Serial.write(m_Head,6);
    Serial.write(LF);    // add an LF 
#else
    if(m_MyFD>0){
        m_pMyPort->write(m_Head,6);
        m_pMyPort->write(&LF,1);    // add an LF 
    }
#endif

    return 1;
}

int Command::Receive()
{
//
// Again because command functionallity is identical to the
// base CommType (see above) we dont need this function
// so calling it is an error
    return -1;
}

//
//------------------------------------------------------------------------------------------
//
CommString::CommString()
{
    m_MyOpt=0;
    m_MyType = MESG_TEXT;
}

CommString::~CommString()
{
// nowt to do, handled in m_String
}

int CommString::SendString(const char* pStr, int len)
{
    int Ret;
    SetString(pStr,len);
    Ret = Send();
    return Ret;
}

int CommString::SetString(const char* pStr, int len)
{
//
    if(len>0){
        m_String.SetString(pStr,0,len);
    }
    else{
        m_String.SetString(pStr);
    }
    m_MyOpt = m_String.Size();
    BuildHead(MESG_TEXT, m_MyOpt);

    return m_MyOpt;
}


int CommString::Send()
{
// sends the string to m_MyFD and returns
// the total number of bytes sent including the header
//
    char* pStr = m_String.GetString();
// 
// ALL strings MUST be terminated with a '\n' so check
// and impose one if it is missing
//
    if(pStr[m_MyOpt-1]!=EndLn){
        pStr[m_MyOpt-1]=EndLn;
    }

#ifdef MY_ARDUINO_BUILD
//
    Serial.write(m_Head,6);
    Serial.write((MYBYTE*)pStr,m_MyOpt);
    Serial.flush();
#else
    m_pMyPort->write(m_Head,6);
    m_pMyPort->write(pStr,m_MyOpt);
    m_pMyPort->flush();
#endif
    return m_MyOpt;
}

#ifdef MY_ARDUINO_BUILD

int  MyReadBytesUntil(char Endch, MYBYTE* pBuf, int Num, int& found)
{
//
// Caution this function will fail if the data stream contains binary data
// It can only be used on text-based data streams as it tests for Endch
//
// This function is my variant of the Arduino function readBytesUntil()
// it will only work with the Arduino Serial port code and hence the compiler directives.
// It is designed to remove inconsistency between Arduino and Linux codes for the same task.
// It will remove 'Endch' from 'pBuf' and it will indicate if this was 'found'
// This simplifies error handling at the higher level when the data is mall-formed.
// It also implements "best practice" of looping over readBytes() if n2>0 
// However it seems to work best when a timeout of 2 msec or more is set
// in the Serial Port initialisation during setup() and returns in one pass
// This has been tested for strings up to 255 bytes
// Clearly more is going on in the readBytes() function and there is probably
// an equivalent loop in that code
//
    int i, n2, count, ret, sum;
    MYBYTE* pC;
    pC = pBuf;
    count = 0;
    found=0;
    ret = 0;
    sum = 0;
    n2 = Num;
//
// available() will block for SerialTimeOut msec
//
    pC = pBuf;

    while( n2>0 ){

        ret = Serial.readBytes((char*)pC,n2);
//
// can happen if the read buffer is exhausted,
// indicate this to the user with found =-1
//
        if(ret<0){
            found=-1;
            break;
        }
//
// test the data read for '\n'
//
        for(i=sum;i<(sum+ret);i++){
            if(pC[i]==Endch){
                found=1;          // if we found the terminator return 
                sum=i+1;          // even if this is < Num bytes
                return sum;
            }
        }
        sum+=ret;
//
// go round for another bite if we did not get it all this pass
//
        n2 = Num - sum;
        pC = pBuf+sum;
        count++;
    }

    return sum;
}

#endif
int CommString::ReceiveAll()
{
//
// Simplifies reading in the whole string when the user
// knows what is present on the stream without needing
// to interrogate
//
    char head[6];
    m_pMyPort->readBytes(head,6);
    DecodeHeader(head);
    return Receive();
}

int CommString::Receive()
{
//
// the base CommType::Receive must have been called or the header Set
// before this routine can be called with m_MyOpt set to a meaningful
// size for the string. The rules for strings are slightly more
// flexible than for data types and minor errors are repaired here
//
    int n, found=0;
    MYBYTE* pStr;
    m_String.ReSize(m_MyOpt);
    pStr = (MYBYTE*)m_String.GetString();
    n = 0;

#ifdef MY_ARDUINO_BUILD

    n = MyReadBytesUntil(EndCh,pStr,m_MyOpt,found);

    if((n<=0) || (found<0) ){
        Serial.print(" A CommString_Receive Error\n");
        pStr[0]=EndCh;
        m_MyOpt=1;
        return -1;
    }
//
// if the number of chars received is less than expected and
// there was no major problem fix it up and adjust down the size
//
    if((n<m_MyOpt)){
        m_MyOpt=n+1;
        pStr[n]=EndCh;
    }else if(n==m_MyOpt){
        if(found==0){
            pStr[n-1] = EndCh;
        }
    }

#else   // Linux

    n = m_pMyPort->readBytesUntil(EndCh,(char*)pStr,m_MyOpt,found);

    if((n<=0) || (found<0) ){
        PrintM(" CommString_Receive Error");
        pStr[0]=EndCh;
        m_MyOpt=1;
        return -1;
    }

    if(n<m_MyOpt){
        sprintf(ErrorBuf,"CommString_Receive Expected %d Got %d found %d",m_MyOpt,n,found);
        PrintM(ErrorBuf);
        m_MyOpt=n+1;
        pStr[n]=EndCh;
    }else if(n==m_MyOpt){
        if(found==0){
            pStr[n-1] = EndCh;
        }
    }

#endif

    return m_MyOpt;
}
//
//--------------------------------------------------------------------------------
// DataComm Class Members 
//
// Golden Rules: 
//  1)    m_ByteLen is inclusive of dataload + terminal '\n'
//  2)    m_ByteLen does not include the header char[6] 
//  3)    m_ByteLen must be set before Send() it is used to write the byte packet to the serial port
//  4)    m_ByteLen must be set prior to Receive() it is used to validate the length of incomming data.
//  5)    There can be no testing for Endch or anything else in a binary data stream
//  6)    The Receive() function requires that the header char[6] data has already been read from the
//        data buffer. m_Head is read at the higher level to control data read processes
// 
// MYBYTE DataComm::SendBuf[COMM_BUF_LIM];
//MYBYTE DataComm::RecvBuf[COMM_BUF_LIM];   

DataComm::DataComm()
{
    m_Size=0;
    m_ByteLen=0;
    m_Identity=0;     
    m_Index=0;
    m_pData = NULL;
    m_pSend = NULL;
    m_pRecv = NULL;
}

DataComm::~DataComm()
{
    if(m_Size>0){
        delete m_pSend;
        delete m_pRecv;
        m_pSend = NULL;
        m_pRecv = NULL;
        m_Size=0;
    }
}
int DataComm::AllocateBuffers(int Size)
{
//
// Dynamically allocate byte buffers 
//
    if(Size>COMM_BUF_LIM){
        PrintM("Warning Size Exceeds Current buffer Limit");
    }
    if(Size<=0){
        m_Size = COMM_BUF_LIM;
    } else{
        m_Size=Size;
    }
    m_pSend = (MYBYTE*)new MYBYTE [m_Size];
    m_pRecv = (MYBYTE*)new MYBYTE [m_Size];
    return 1;
}
void DataComm::ResetForSend()
{
//
// This function MUST be called at the start of the derived class
// Send() before any other data is committed to the buffer
//
//    m_pData = SendBuf;
    m_pData = m_pSend;
    m_Index=0;
    WriteStringToByteStream(m_Head);
    m_Index = 6;
}

void DataComm::ResetForReceive()
{
// This function MUST be called at the start of the
// derived class Receive() function
//
//    m_pData = RecvBuf;
    m_pData = m_pRecv;
//    WriteStringToByteStream(m_Head);
    m_Index = 0;   // set to the start Index of the actual dataload
}

#ifdef MY_ARDUINO_BUILD

int DataComm::readDataBytes(MYBYTE* pbuf, int len)
{
//
// Use this ONLY for Binary data stream
// It will read a fixed length of byte data
// it will not look for a special terminator characters
// in the data stream so it may block if count<length
//
    int count=0;
    count = Serial.readBytes(pbuf,len);
    
    return (count>0) ? count: -1;
}

#endif

int DataComm::Send()
{
// This function cannot be called directly but only at the END
// of a derived class Send() send after data has been committed
// to the send byte buffer
//
// Actually sends The Data packet Over the Serial link
// at this point m_Index should be the total length of the string
// the length of the dataload (m_ByteLen) plus the length of m_Head (6)
//
    if(m_Index!=(m_ByteLen+6)){
        PrintM("DataComm_Send Message length discrepancy");
    }

    #ifdef MY_DEBUG
        printf("# %s> DataComm-Send Type %d, Opt %d, Index %d, FD %d\n",MsgSrc,m_MyType,m_MyOpt,m_Index,m_pMyPort->GetFD());
        PrintDecodeString(m_pData,m_Index);
    #endif

#ifdef MY_ARDUINO_BUILD
    Serial.write(m_pData,m_Index);
    Serial.flush();
#else
    m_pMyPort->write(m_pData,m_Index);
    m_pMyPort->flush();


#endif
    return m_Index; // total length including head
}     

int DataComm::Return()
{
    m_pData = m_pRecv;
    m_Index=0;
    WriteStringToByteStream(m_Head);
    m_Index = 6+m_ByteLen;
    return Send();
}

int DataComm::ReceiveAll()
{
    return -1;
}

int DataComm::Receive()
{
//
// The 6 char m_Head MUST already have been read before this function
// is called. It should be called at the beginning of the derived classes
// Receive() function to get the data from the serial port. The first 4 bytes 
// will be two SHORTIs defining m_ByteLen and m_Identity of this data packet.
// Having read this, m_ByteLen Bytes are read into RecvBuf with no 
// data conversions. This can then be decoded in the derived classes
//
// Need to think about efficient Re-direction of an incomming buffer!
//
// Variable length data packets are not possible due to finite 
// resource limitations of Arduino and other simple embedded systems
// so the caller MUST set m_ByteLen before this function is called.
// The function can fail for a number of reasons
//   return         Reason
//   -1             m_Bytelen not set
//   -2             Failed to read header bytes from bytestream
//   -3             Length read != expected
//   -4             Identity of stream != m_Identity
//   -n             Number of bytes read did not match
//   count          Sucess = Number of bytes in the dataload
//
//
    int n;
    int Ident, Len, count;
    MYBYTE* pC;

    ResetForReceive();
//
    if(m_ByteLen<=0){
      return -1;
    }
//
    count = 2*sizeof(SHORTI);

// read the length of the buffer and Ident
#ifdef MY_ARDUINO_BUILD
    n = readDataBytes(m_pData,count);
#else
    n = m_pMyPort->readDataBytes(m_pData,count);
#endif
    if(n!=count){
      return -2;
    }
    Len = readShort();
    if(Len!=m_ByteLen){
        return -3;
    }
//
// we only care about Ident if m_Identity is set by the caller
// it is a means of having multiple different DataComm objects of the same type 
// read back as distinct objects, probably redundant in most case
//
    Ident = readShort();
    if(m_Identity>0){
      if(Ident!= m_Identity){
        return -4;
      }
    }
//
// Get the rest of the data body (note m_ByteLen is self inclusive so we discount it from the remainder to read)
//
    pC = m_pData + count;
    count = m_ByteLen - count;
//
// Read the data load into the receive buffer
//
#ifdef MY_ARDUINO_BUILD
    n += readDataBytes(pC,count);
#else
    n += m_pMyPort->readDataBytes(pC,count);
 #endif
    if( n<m_ByteLen ){
        return -n;
    }
    count = n - 2*sizeof(SHORTI) - 1;
    return count;   // return the number of bytes in the dataload
}
//
// All of these functions use m_Index as a state variable into SendBuf
// and return the number of bytes used.
// There is the obvious possibility of a reset failure and an
//
int DataComm::WriteShortToByteStream(const SHORTI iVal)
{
	int i, n = sizeof(SHORTI);
	MYBYTE *pV;
    pV = (MYBYTE*)(&iVal);
	for(i=0;i<n;i++){
		m_pData[m_Index+i] = (MYBYTE)pV[i];
	}
    m_Index+=n;
    return n;
}

int DataComm::WriteDWordToByteStream(const MYDWORD lVal)
{
	int i, n = sizeof(MYDWORD);
	MYBYTE *pV;
	pV = (MYBYTE*)(&lVal);
	for(i=0;i<n;i++){
		m_pData[m_Index+i] = (MYBYTE)pV[i];
	}
    m_Index+=n;
    return n;

}

int DataComm::WriteFloatToByteStream(const float fVal)
{
	int i, n = sizeof(float);
	MYBYTE *pV;
	pV = (MYBYTE*)(&fVal);
	for(i=0;i<n;i++){
		m_pData[m_Index+i] = (MYBYTE)pV[i];
	}
    m_Index+=n;
	return n;
}

int DataComm::WriteDoubleToByteStream(const double dVal)
{
	int i, n = sizeof(double);
	MYBYTE *pV = (MYBYTE*)(&dVal);
	for(i=0;i<n;i++){
		m_pData[m_Index+i] = (MYBYTE)pV[i];
	}
    m_Index+=n;
	return n;
}

int DataComm::WriteStringToByteStream(const MYBYTE* pStr)
{
    int i, len;
    len = strlen((char*)pStr);
    if(len <=0 ){
        return -1;
    }

    for(i=0;i<len;i++){
	m_pData[m_Index+i] = (MYBYTE)pStr[i];
    }
    m_Index+=len;
    return len;
}

int DataComm::WriteEndCharToByteStream()
{
    m_pData[m_Index] = '\n';
    m_Index++;
    return 1;
}
//
// All of these functions use m_Index as a state variable into the RecvBuf
// Obvious error is failure to Reset and possible buffer overrun
// The user should test for this it is not done here
// an obvious test is if(m_Index==m_ByteLen) at the end
//

SHORTI  DataComm::readShort()
{
    SHORTI ival;
    MYBYTE* pC = m_pData + m_Index;
    SHORTI* pIval = (SHORTI*)pC;
    ival = *pIval;
    m_Index += sizeof(SHORTI);
    return ival;
}

MYDWORD DataComm::readDWord()
{
    MYDWORD ival;
    MYBYTE* pC = m_pData + m_Index;
    MYDWORD* pIval = (MYDWORD*)pC;
    ival = *pIval;
    m_Index += sizeof(MYDWORD);
    return ival;
}

float DataComm::readFloat()
{
    float fVal;
    MYBYTE* pC = m_pData + m_Index;
    float* pFval = (float*)pC;
    fVal = *pFval;
    m_Index += sizeof(float);
    return fVal;
}

double  DataComm::readDouble()
{
    double dVal;
    MYBYTE* pC = m_pData + m_Index;
    double* pDval = (double*)pC;
    dVal = *pDval;
    m_Index += sizeof(double);
    return dVal;
}

FString DataComm::readString(int Len)
{
    int i;
    char c;
    FString Str;

    Str.ReSize(Len);
    for(i=0;i<Len;i++){
        c = (char)m_pData[m_Index+i];
        Str.SetChar(i,c);
    }
    m_Index+=Len;
    return Str;
}

//
//-----------------------------------------------------------------------------------------------
//
IntDataComm::IntDataComm(SHORTI* pData, int Len, int Ident)
{
    if( Len>0){
        SetData(pData,Len,Ident);
    } else{
        m_DataLength = 0;
        m_pVec = NULL;
        m_ByteLen = 0;
    }
    BuildHead(MESG_DATA,INT_DATA_COMM);
}

IntDataComm::~IntDataComm()
{
    m_DataLength=0;
    m_pVec=NULL;
}

int IntDataComm::SetData(SHORTI* pData, int Len, int Ident)
{
//
// this function must be called before both sending and receiving
// data with a pointer (pData) to an array of the correct Len
// if Ident is set >0 it will be used to discriminate between
// different IntDataComm objects otherwise it will be egnored
// 
    m_DataLength = (SHORTI)Len;
    m_pVec = pData;
    m_ByteLen = (m_DataLength + 2 )*sizeof(SHORTI) + 1;
    AllocateBuffers(m_ByteLen);   
    m_Identity=Ident;
    return Len;
}

int IntDataComm::Send()
{
//
// This MUST be called last in the derived class Send()
// function it builds and sends the byte stream as one string
//    
    int i, count;

    ResetForSend();
//
// Byte length of the dataload
//
    m_ByteLen =(m_DataLength + 2 )*sizeof(SHORTI) + 1;

// Build the bytestream to send

    count = WriteShortToByteStream(m_ByteLen);
    count += WriteShortToByteStream(m_Identity);

    for(i=0;i<m_DataLength;i++){
        count += WriteShortToByteStream(m_pVec[i]);
    }

    count += WriteEndCharToByteStream();
//###
//    Print();
//###
    DataComm::Send();
    return count;
} 

int IntDataComm::ReceiveAll()
{
//
// Simplifies reading in the whole string when the user
// knows what is present on the stream without needing
// to interrogate
//
    char head[6];
    m_pMyPort->readBytes(head,6);
    DecodeHeader(head);
    return Receive();
}
int IntDataComm::Receive()
{
//
// DataComm::Receive includes byte checks and Identity
// count is simply a checksum returns -ve if errors
//
    int i, count;

    count = DataComm::Receive();
    if(count<0){
        sprintf(ErrorBuf,"Failed IntDataComm::Receive Count = %d",count);
        PrintM(ErrorBuf);
        return count;
    }
//
    count=0;    
    for(i=0;i<m_DataLength;i++){
        m_pVec[i] = readShort();
        count ++;
    }

    return count; // return the number of data items
}

void IntDataComm::Print(FILE* pFile)
{
    int i;
//    SHORTI out;
    if(pFile){
        fprintf(pFile,"# %s> IntDataComm, Ident %d, Size %d,\n",MsgSrc,m_Identity,m_DataLength);
        fprintf(pFile,"# %s> Data ",MsgSrc);
        for(i=0;i<m_DataLength;i++){
            fprintf(pFile," [%d] = %d, ",i,m_pVec[i]);
        }
        fprintf(pFile,"\n");
    } else {
        printf("# %s> IntDataComm, Ident %d, Size %d,\n",MsgSrc,m_Identity,m_DataLength);
        printf("# %s> Data ",MsgSrc);
        for(i=0;i<m_DataLength;i++){
            printf(" [%d]=%d, ",i,m_pVec[i]);
        }
        printf("\n");
//###
/*
        i = GetIndex();
        printf("# %s> Outgoing Index %d \n",MsgSrc,i);
        ResetForSend();
        for(i=0;i<6;i++){
            out = readShort();
            printf(" Out[%d] = %d,",i,(int)out);
        }
        printf("\n");
*/
//###
    }
}
//
//--------------------------------------------------------------------------------
//
TimerComm::TimerComm(int ID)
{
    BuildHead(MESG_DATA,TIMER_COMM);
    SetIdentity((SHORTI)ID);
    m_Tick=0;
    m_Tock=0;
    m_ByteLen = 2*sizeof(SHORTI) + 2*sizeof(MYDWORD)+1;
#ifndef MY_ARDUINO_BUILD
    AllocateBuffers(m_ByteLen);
#endif
}

#ifndef MY_ARDUINO_BUILD

void  TimerComm::StartTimer()
{
//
// resets m_Start
//
    gettimeofday(&m_Start, NULL);
}

MYDWORD TimerComm::MyMillis()
{
//
// This function replicates the functionallity of the Arduino millis()
// function which returns the elapsed time in milliseconds since the above
// StartTimer() function was called. This is slightly different to the Arduino
// implementation which starts from when the device is turned on, call behaviour 
// and usage is otherwise identical
//
	long int usecs, secs;
	MYDWORD msec;  

	gettimeofday(&m_Now,NULL);

	secs  = m_Now.tv_sec  - m_Start.tv_sec;
    usecs = m_Now.tv_usec - m_Start.tv_usec;

    msec = (MYDWORD)(usecs/1000.0 + secs*1000);

	return msec;
}

MYDWORD TimerComm::MyMicros()
{
//
// returns time elapsed since timer started in micro-seconds
// more care is needed since it will
	long int usecs, secs;
	MYDWORD  usec;  

	gettimeofday(&m_Now,NULL);

	secs  = m_Now.tv_sec  - m_Start.tv_sec;
    usecs = m_Now.tv_usec - m_Start.tv_usec;
    m_Sign = usecs;
    
    usec = (MYDWORD)(usecs + secs*1000000);

	return usec;
}

#endif

void TimerComm::Tick()
{
#ifdef MY_ARDUINO_BUILD
    m_Tick = millis();
#else
    m_Tick = MyMillis();
#endif
}

int  TimerComm::Tock()
{
// returns the time delta since Tick() 
// or -1 if Tick() not previously set
// it will reset Tick to 0 to so can
// only be used in sequence with Tick()
//
    int delta;

    if(m_Tick>0){

#ifdef MY_ARDUINO_BUILD
        m_Tock = millis();
#else
        m_Tock = MyMillis();
#endif
        delta = (int)(m_Tock-m_Tick);
        m_Tick=0;
        return delta;
    } else {
        return -1;
    }
}

void TimerComm::MuTick()
{
#ifdef MY_ARDUINO_BUILD
    m_Tick = micros();
#else
    m_Tick = MyMicros();
#endif
}

int  TimerComm::MuTock()
{
// returns the time delta since Tick() 
// or -1 if Tick() not previously set
// it will reset Tick to 0 to so can
// only be used in sequence with Tick()
//
    int delta;

    if(m_Tick>0){

#ifdef MY_ARDUINO_BUILD
        m_Tock = micros();
#else
        m_Tock = MyMicros();
#endif
        delta = int(m_Tock-m_Tick);
        m_Tick = 0;
        return delta;
    } else {
        return -1;
    }
}

int TimerComm::Send()
{
    int count;
// build a bytestream of the data to the buffer
    ResetForSend();
    count  = WriteShortToByteStream(m_ByteLen);
    count += WriteShortToByteStream(m_Identity);
    count += WriteDWordToByteStream(m_Tick);
    count += WriteDWordToByteStream(m_Tock);
    count += WriteEndCharToByteStream();

    DataComm::Send();
    return count;
}


int TimerComm::Receive()
{
//
// Receive a timer input we already have the header
//
    int count;
    count = DataComm::Receive();
    if(count<0){
        sprintf(ErrorBuf," TimerComm::Receive Error Count = %d",count);
        PrintM(ErrorBuf);
        return count;
    }
    m_Tick = readDWord();
    m_Tock = readDWord();
//
    return (int)m_Identity;    // return this timers ID
}

int TimerComm::Return()
{
    return DataComm::Return();
}
//
//-----------------------------------------------------------------------------------------------
//
FloatDataComm::FloatDataComm(float* pVec, int Len, int Ident)
{
  if(Len>0){
      SetData(pVec,Len,Ident);
  } else {
      m_DataLength = 0;
      m_Identity=0;
      m_pVec = NULL;
  }
  BuildHead(MESG_DATA,FLOAT_DATA_COMM);
}

FloatDataComm::~FloatDataComm()
{
    m_DataLength=0;
    m_pVec=NULL;
}

int FloatDataComm::SetData(float* pSource, int Len, int Ident)
{
//
// this function must be called before both sending and receiving
// data with a pointer (pData) to an array of the correct Len
// if Ident is set >0 it will be used to discriminate between
// different FloatDataComm objects otherwise it will be egnored
// 
    m_pVec = pSource;  
    m_DataLength=Len;
    m_Identity=Ident;
    m_ByteLen = m_DataLength*sizeof(float) + 2*sizeof(SHORTI) + 1;
    AllocateBuffers(m_ByteLen);
    return m_ByteLen;
}

int FloatDataComm::Send()
{
    int i, count;

    ResetForSend();

// Build the message to send

    count = WriteShortToByteStream(m_ByteLen);
    count+= WriteShortToByteStream(m_Identity);

    for(i=0;i<m_DataLength;i++){
        count += WriteFloatToByteStream(m_pVec[i]);
    }

    count += WriteEndCharToByteStream();

    DataComm::Send();
    return count;
} 

int FloatDataComm::ReceiveAll()
{
//
// Simplifies reading in the whole string when the user
// knows what is present on the stream without needing
// to interrogate
//
    char head[6];
    m_pMyPort->readBytes(head,6);
    DecodeHeader(head);
    return Receive();
}

int FloatDataComm::Receive()
{
//
// This will fail if m_pVec does not point to
// an existing data vector of the correct size
// and m_DataLength is not set
//
    int i, count;

    if(DataComm::Receive()<0){
        return -1;
    }

    count = 0;
    for(i=0;i<m_DataLength;i++){
        m_pVec[i] = readFloat();
        count++;
    }
    return count;
}

void FloatDataComm::Print(FILE* pFile)
{
    int i;
//    SHORTI out;
    if(pFile){
        fprintf(pFile,"# %s> FloatDataComm, Ident %d, Size %d,\n",MsgSrc,m_Identity,m_DataLength);
        fprintf(pFile,"# %s> Data ",MsgSrc);
        for(i=0;i<m_DataLength;i++){
            fprintf(pFile," [%d] = %.5f, ",i,m_pVec[i]);
        }
        fprintf(pFile,"\n");
    } else {
        printf("# %s> FloatDataComm, Ident %d, Size %d,\n",MsgSrc,m_Identity,m_DataLength);
        printf("# %s> Data ",MsgSrc);
        for(i=0;i<m_DataLength;i++){
            printf(" [%d]=%.5f, ",i,m_pVec[i]);
        }
        printf("\n");
//###
/*
        i = GetIndex();
        printf("# %s> Outgoing Index %d \n",MsgSrc,i);
        ResetForSend();
        for(i=0;i<6;i++){
            out = readShort();
            printf(" Out[%d] = %d,",i,(int)out);
        }
        printf("\n");
*/
//###
    }
}

//
//-----------------------------------------------------------------------------------------------
//
//
DepthDataComm::DepthDataComm(MYWORD* pVec, MYDWORD Len)
{
  if((Len>0) && (pVec!=NULL) ){
      SetData(pVec,Len);
  } else {
      m_DataLength = 0;
      m_pVec = NULL;
  }
  BuildHead(MESG_DATA,FLOAT_DATA_COMM);
}

DepthDataComm::~DepthDataComm()
{
    m_DataLength=0;
    m_pVec=NULL;
}

int DepthDataComm::SetData(MYWORD* pSource, MYDWORD Len)
{
//
// reference a valid source for both incomming and outgoing
// pSource is a memory pointer to exactly 
//      Len*sizeof(MYWORD) bytes of contiguous memory
//
// Note also that this class uses a separate memory buffer m_pVec
// NOT the base class m_pData, this should be resolved
//
    m_pVec = pSource;  
    m_DataLength=Len;
    m_LongByteLen = m_DataLength*sizeof(MYWORD) + sizeof(MYDWORD) + 1;

    return (int)m_LongByteLen;
}

int DepthDataComm::Send()
{
    int count, len, ret;
//###    ResetForSend(); not used
 
    BuildHead(MESG_DATA,DEPTH_DATA_COMM);

// we override the basic operation to write directly to FD
    count  = m_pMyPort->write(m_Head,sizeof(m_Head));
    count += m_pMyPort->write(&m_LongByteLen,sizeof(m_LongByteLen));

    len = m_DataLength*sizeof(MYWORD);
    ret = m_pMyPort->write(m_pVec,len);
    if(ret==-1){
        perror("DepthDataComm write error detected");
        return -1;
    }else{
        count +=ret;
        count += m_pMyPort->write(&EndLn,1);
    }
//### not used    DataComm::Send();
    return count;
}

int DepthDataComm::ReceiveAll()
{
//
// Simplifies reading in the whole string when the user
// knows what is present on the stream without needing
// to interrogate
//
    char head[6];
    m_pMyPort->readBytes(head,6);
    DecodeHeader(head);
    return Receive();
}

int DepthDataComm::Receive()
{
//
// This will fail if m_pVec does not point to
// an existing data vector of the correct size
// it is also assume that 
    MYDWORD mbytes;
    int Len, count;
    char ret;
    count = read(m_MyFD,&mbytes,sizeof(MYDWORD));
    Len = (mbytes - sizeof(MYDWORD) - 1)/sizeof(MYWORD);

    if(Len>(int)m_DataLength){
        sprintf(ErrorBuf,"DepthDataComm Read Error Len %d > m_DataLength %d",Len,m_DataLength);
        PrintM(ErrorBuf);
    }

    count += read(m_MyFD,m_pVec,m_DataLength*sizeof(MYWORD));
    count += read(m_MyFD,&ret,1);
    return count;

}


RobSeqCmd::RobSeqCmd()
{
    m_SeqNo=0;
    Device = 0;
    Channel=0;
    Step=0;
    m_Active=0;
    BuildHead(MESG_DATA,SEQUENCE_COMM);
//
// Byte length of the dataload
//
    m_ByteLen = (4 + 2 )*sizeof(SHORTI) + 1;
    
    AllocateBuffers();
}

int RobSeqCmd::Send()
{
    int count;

    ResetForSend();


// Build the bytestream to send

    count = WriteShortToByteStream(m_ByteLen);
    count += WriteShortToByteStream(m_Identity);

    count += WriteShortToByteStream(m_SeqNo);
    count += WriteShortToByteStream(Device);
    count += WriteShortToByteStream(Channel);
    count += WriteShortToByteStream(Step);

    count += WriteEndCharToByteStream();

    DataComm::Send();
    return count;
}

int RobSeqCmd::Receive()
{
// DataComm::Receive includes byte checks and Identity
// count is simply a checksum returns -ve if errors
//
    int count;

    count = DataComm::Receive();
    if(count<0){
        sprintf(ErrorBuf,"Failed RobSeqCmd-Receive Count = %d",count);
        PrintM(ErrorBuf);
        return count;
    }
    count=0;

    m_SeqNo = readShort();
    count++;

    Device  = readShort();
    count++;

    Channel = readShort();
    count++;

    Step = readShort();
    count++;
    return count;
}

void RobSeqCmd::Print()
{
    sprintf(ErrorBuf,"RobSeqCmd SeqNo %d, Device %d, Channel %d, Step %d ",m_SeqNo,Device,Channel,Step);
    PrintM(ErrorBuf);
}


int RobSeqCmd::FromFile(FILE* pFile)
{
    FString Line;
    if(Next()){
        FindNextLine(pFile, Line);
        return FromString(Line);
    } else {
        return 0;
    }
}

int RobSeqCmd::FromString(const char* pData)
{
// Convert the string to the next SeqCmd
// Note that Next() must be called before this
// function
//
    FString Number, OutNumStr, InNumStr;
    int Idx=0;
    if(strcasestr(pData,"end")){
		PrintM(" End of Script File");
        return 0;
    }
// currently there are only 2 devices 
// and some of the members are zero so we use
// logic here to simplify the file format
    if(strcasestr(pData,"tra")){
        Device = (SHORTI)1;
        Channel = (SHORTI)0;
        FindNextNumber(pData,Number,Idx);
        Step = (SHORTI)STR_2_I(Number);
    }else if(strcasestr(pData,"cam")){
        Device = (SHORTI)3;
        Channel = (SHORTI)1;
        FindNextNumber(pData,Number,Idx);
        Step = (SHORTI)STR_2_I(Number);
    } else if(strcasestr(pData,"car")){
        Device = (SHORTI)2;
        FindNextNumber(pData,Number,Idx);
        Channel=(SHORTI)STR_2_I(Number);  
        FindNextNumber(pData,Number,Idx);
        Step=(SHORTI)STR_2_I(Number);  
    }
    
    return m_SeqNo;
}

int RobSeqCmd::Check(RobSeqCmd& Return)
{
    int Error;

    if(!Active()){
        sprintf(ErrorBuf," Sequence No %d is not active",m_SeqNo);
        PrintM(ErrorBuf);
        return 0;
    }
    if(Return.Device == Device){
        if(Return.m_SeqNo == m_SeqNo){
            Error=0;
        } else {
            Error=1;
        }
    } else {
        Error=1;
    }
    if(Error>0){
        sprintf(ErrorBuf," Error SeqNo %d Received From Device %d != Posted SeqNo %d to Device %d",Return.m_SeqNo, Return.Device, m_SeqNo, Device );
        PrintM(ErrorBuf);
        return 0;
    } else {
        Done();
        return 1;
    }
}

void PrintM(const char* pMess)
{
//
// This function is ONLY designed to work with Text Messages
// these can be either Errors or informational messages
// It provides a simple routing mechanism based on which
// output ports are available.
//
// The messages are also decorated with the MsgSrc info
// however if pMess already contains a # it sent as is
//
// It also requires that MsgSrc, HaveScreen, Msg_Pipe_FD 
// or Msg_WIFI_FD should be globally set in the calling
// program as needed otherwise the messages will be lost
//
// A weakness is that it does not deal with anything other
// than simple text so use sprintf to prebuild messages 
// containing complex numeric and text data
// It uses local buffer OutPr to avoid a possible
// double reference to a global buffer and/or overwriting 
// of output strings, as they the can be repackaged
//
    char OutPr[MAX_TEXT_STRING] = {0};
    char* pOut;
//
// decorate with MsgSrc if not alread done
// 
    if(strstr(pMess,"#")){
        pOut = (char*)pMess;
    } else{
        sprintf(OutPr,"# %s> %s\n",MsgSrc,pMess);
        pOut = OutPr;
    }
//
// Route to output print messages 
// they always go up the chain to the device
// that has the screen and then preferentially
// WiFi and finally pipe 
//
#ifdef MY_ARDUINO_BUILD
    Serial.println(pOut);
#else
    int len = strlen(pOut);

    if(strstr(pOut,"\n")==NULL){
        pOut[len-1] = '\n';
    }

    if(HaveScreen){
        printf("%s",pOut);
    } if(Msg_WIFI_FD>0){
        ::write(Msg_WIFI_FD,pOut,len);
    } else if(Msg_Pipe_FD>0){
        ::write(Msg_Pipe_FD,pOut,len);
    }
#endif
}

void PrintDecodeString(MYBYTE* pData, int Len)
{
//
// This decodes any string of text or binary data
// into a printable string. All bytes are converted
// into representable ASCII characters according to 
// the following rules anything  t <  32
// is shifted into t+48 with the exception of \r & \n
// which are translated into r and n respectively 
// Anything greater than 126 is translated into #
//
// Obviously binary data will appear as
// incoherent nonsense but we should at least see
// what is on the buffer 
// 
// The function is designed to be used to interrogate
// what is actually sent and/or received on a serial
// transmission buffer.
//
// The function assumes that Len is valid for the
// supplied pData
//
    int i;
    MYBYTE t;
    char c;
    if(Len==0){
        Len=strlen((const char*)pData);
    }

    for(i=0;i<Len;i++){
        t = pData[i];
        if(t==10){
            t=110;      // \n = n
        } else if(t==13){
            t=114;      // \r = r
        } else if(t<32){
            t=t+48;
        } else if(t>126){
            t=35;           // >126 = #
        }
        c =char(t);
        TextBuf[i] = c;
    }
    if(TextBuf[Len-1] != '\n'){
        TextBuf[Len-1] = '\n';
    }
    TextBuf[Len] = '\0';
    if(HaveScreen){
        printf("# %s> Decode String %s",MsgSrc,TextBuf);
    }
}
