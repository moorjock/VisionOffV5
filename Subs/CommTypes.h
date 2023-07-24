//
// <CommTypes.h>
//
// Contains classes for common commands and data objects to be sent across the serial link
// between any linux machine and any Arduino. 
// There are some implementation differences that cannot be reconciled with #ifdef directives
//
// This file is largely the same on both sides it needs to be put in the Arduino
// /projectname/ directory along with the projectname.ino script
// These CommTypes are used both sides of the link.
//
// The message format used is as follow :Annn!ddddd\n
// The interpretation varies depending on the type
// The rule for all types is that the first 6 readable/printable characters contain a capital letter
// defining the message type and an integer option code nnn this option code depends on the type
// valid types are:
//      C       A command in this case nnn defines the command index and there is no dddd part
//      T       A simple text message in this case nnn is the length of the text
//              string FOLLOWING and not including the 6 character header but including 
//              a terminating LF = '\n'. This MUST be placed as the last character in the string 
//      E       An Error in this case nnn specifies an error code for use in a switch/case block 
//              this is followed by an error dependant diagnostic text with information for the user
//              With the same rule regarding '\n' placement in the error message 
//      D       A data packet/structure, The nnn part defining the data type for use in a switch/case
//              the next 2 bytes following the ! in the body of the message defines the length in 
//              bytes of the data load. Any class or struct data can be passed this way.

// The serial communication rules are:
//
// 0) The first 6 characters of Every message type are ALWAYS readable TEXT 
//    this simple rule allows for great flexibility outside of the rules given below.
//
// 1) Commands are 6 characters followed by a '\n' i.e. 7 chars in total i.e. ":C  1!\n" = init
//
// 2) Text messages are 6 characters in the form ":Txxm!" followed by a text string of m bytes 
//    terminated by a '\n' which must be included in the character count m.
//    It is BAD practice to include control characters ':' or '!' in the text body.
//
// 3) Data messages are of the form :Dxxm!dddd\n where the index m defines the type of data
//    data items with an index m less than 10 are single values sent as text these break the rules
//    regarding binary data handling
//
// 4) Data items with an index 10 and above are binary data where the index m defines the type of dataload.
//    This is application specific but generic IntDataComm and FloatDataComm types allow for defined length
//    vectors of those basic types. 
//
// 5) Other more complex implementation/class-specific data types can be derived as needed however 
//    based on experience this practice is burdensome, inflexible and discouraged.
//
// This is meant to be a simple, lightweight, flexible and easy to use protocol. 
// However a word of CAUTION: It is very easy to break this messaging system especially when 
// mixing text and binary data in the same link which can leading to unexpected blocking or 
// other unexpected data mishandling. Using consistent data packets which obey the above 
// 6 character prefix rules is the best safeguard against such problems.
// 
//
// Note also that it is not a good idea to include other functionallity in these classes since they
// are shared across computers. Device-specific functionallity is best placed in a separate class
// or code file using the functionality in these classes to send/retrieve data.
// It does also require that if a new class is to be transmitted/recieved then a comms class
// needs to be defined here, though they are quite lightweight and not difficult to define
//
// A golden rule for text messages is do not include any of the control characters 
// i.e. ':' or '!' in the message body as it fools the front end interpreter and 
// the message will be lost
//-----------------------------------------------------------------------------------
//  Version:    2.1
//  Changes:    Added ReceiveAll() to selected CommTypes modified CommString::Receive
//  Date:       30 April 2021
//  Programmer:	A. Shepherd
//-----------------------------------------------------------------------------------
//  Version:    2.0
//  Changes:    Dynamic Allocation of DataComm arrays RobSeqCmd class and PrintM added
//  Date:       30 April 2021
//  Programmer:	A. Shepherd
//-----------------------------------------------------------------------------------
//  Version:    1.0
//  Changes:    From ServoDrive Laptop Ubuntu 18.04 Version
//  Date:       9 March 2021
//  Programmer:	A. Shepherd
//-----------------------------------------------------------------------------------
//
// different types of incoming and outgoing messages
//
#ifndef COMMTYPES_H
#define COMMTYPES_H

#define MESG_UNKNOWN 0
#define MESG_COMMAND 1
#define MESG_TEXT    2
#define MESG_ERROR   3
#define MESG_DATA    4
//
// values less than 10 are processed as simple single data items
// case dependent on specific application
// the following 3 items are generic
//
    #define TIMER_COMM          10
    #define INT_DATA_COMM       11
    #define FLOAT_DATA_COMM     12
    #define DEPTH_DATA_COMM     14
    #define SEQUENCE_COMM       15
    #define REALTIME_COMM       16

//  remain to be implemented
//  #define COLOUR_DATA_COMM    20
//  #define MESG_VIDEO          21

#define RT_MODE  1
#define SEQ_MODE 2

// these are the principle command codes
#define RUN_FAIL  0
#define RUN_INIT  1
#define RUN_START 2
#define RUN_STOP  3
#define RUN_FAULT 4
#define RUN_WAIT  5
#define RUN_RESET 6
#define RUN_TEST  7
#define RUN_VERSION 8
#define RUN_END   9
#define RUN_TIMER 10
#define RUN_PARAMS 11
#define RUN_CONNECT 12
#define RUN_ACK     13

#define HEAD_SIZE 6

#ifdef MY_ARDUINO_BUILD

int  MyReadBytesUntil(char Endch, MYBYTE* pBuf, int Num, int& found);

#else
 class Serial;
#endif


class CommType
{

protected:
    int             m_MyFD;

#ifndef MY_ARDUINO_BUILD
    Serial*         m_pMyPort;   // the data channel to which this object is attached 
                                 // it can be a Serial port named Pipe or Socket and can be changed 
                                 // dynamically provided said port is open for business
#endif
protected:
    SHORTI         m_MyType;     // class members are defined using MyDataTypes.h ONLY. To save needless
                                 // data retyping local storage types char, int are used internally
    SHORTI         m_MyOpt;      // meaning varies with m_MyType type 
    MYBYTE         m_Head[6];    // the packet head is always 6 readable chars

    int BuildHead(int Type, int Opt);

public:

    CommType();

#ifndef MY_ARDUINO_BUILD
    void SetCommsPort(Serial* pPort=NULL)
        {
            m_pMyPort = pPort;
            m_MyFD = m_pMyPort->GetFD();
        }
    
    Serial* GetCommsPort(){return m_pMyPort;}

    int  GetFD(){return m_pMyPort->GetFD();}
#endif

    int SetHead(int type, int Opt);

    int SetHead(CommType& Base);    // Copy Constructor

    int DecodeHeader(const char* pStr); // from an incomming string

    virtual int Send(){return 0;}

    virtual int Receive(){return -1;}

    virtual int ReceiveAll(){return -1;}

    int GetType(){ return m_MyType;}
    int GetOpt(){ return m_MyOpt;}
};

//
// The command class is used to send and recieve short 6 character messages
// to/from the connected serial port, the command can then processed on reciept
// that activity is not part of this class which is just ensuring 
// robust recipt/delivery
//
class Command : public CommType
{

public:

    Command(int m_Command=0);
    
    Command(CommType& Base);   // copy constructor

// some standard commands
    void InitDevice();        
    void Start();
    void Stop();
    void Reset();
    void Version();

    int SetCommand(int com);

    int SetCommand(CommType& Base);

    int GetCommand(){ return GetOpt();}

    int Send();     // send to the com port

    int Receive();  // read from the com port

};

//
// Typically used to send a long text string this may be a file header
// or some other text message (like errors) from either end

class CommString : public CommType
{
protected:

    FString       m_String;        // String object holding the data

public:

    CommString();
    ~CommString();

    int SetString(const char* pStr, int len=0); // set up this string

    char*   GetString(){return m_String.GetString();}
    
    int Size(){ return m_String.Size();}

    int SendString(const char* pStr, int len=0);

    int Send();

    int Receive();

    int ReceiveAll();   // Gets the whole string including the head

};
//
// DataComm is an abstract class used to manage
// Sending and Receiving of other Data classes derived 
// from this one, Don't use this directly derive from it
//
// The following buffer is limited by Arduino UNO memory constraints
// It could with caution be extended using a DUE or MEGA
// 
#define COMM_BUF_LIM 256

class DataComm : public CommType
{
    SHORTI      m_Index;        // m_Index is intentionally private to this class
    MYBYTE*     m_pSend;        // Dynamically allocated send buffer
    MYBYTE*     m_pRecv;        // Dynamically allocated receive buffer

protected:  // class members are accessible by derived classes

    SHORTI      m_ByteLen;      // The length of the byte stream including '\n'
    SHORTI      m_Identity;     // The identity/use of this data can be zero
    MYBYTE*     m_pData;        // The buffer Home position
    int         m_Size;         // size of the dynamic buffers
    
//    static      MYBYTE SendBuf[COMM_BUF_LIM];
//    static      MYBYTE RecvBuf[COMM_BUF_LIM];   

    SHORTI      GetIndex(){return m_Index;}

    int         AllocateBuffers(int Size=0);

public:
    int ByteSize(){return m_ByteLen;}

    virtual int Size(){return 0;}

    void SetIdentity(int ID){m_Identity=ID;}

    int  GetIdentity(){return(int)m_Identity;}

protected:

    DataComm();

    ~DataComm();

    void ResetForSend();
    void ResetForReceive();
//
// The following functions are used to write data types to the bytestream
// Currently these are 
//
public:
#ifdef MY_ARDUINO_BUILD
    int readDataBytes(MYBYTE* pbuf, int len); 
#endif

    virtual int Send();     // send the byte-stream over the comms link

    virtual int ReceiveAll();   // dont implement at this level

    virtual int Receive();  // receive the byte stream body (minus head)

    virtual int Return();

protected:

    int WriteShortToByteStream(const SHORTI iVal);

    int WriteDWordToByteStream(const MYDWORD lVal);

    int WriteFloatToByteStream(const float fVal);

    int WriteDoubleToByteStream(const double dVal);

    int WriteStringToByteStream(const MYBYTE* pStr);

    int WriteEndCharToByteStream();

    SHORTI   readShort();

    MYDWORD  readDWord();

    float    readFloat();

    double   readDouble();

    FString  readString(int Len);

};
//
// IntDataComm is a generic transport for integer vectors 
// to allow application-specific sub-classing an identity
// is set, note that integers are 32-bit on all machines
//
class IntDataComm : public DataComm
{
    SHORTI      m_DataLength;      // The number of data items
    SHORTI*     m_pVec;            // The data and we don't own it

public:

    IntDataComm(SHORTI* pData=0, int Len=0, int Ident=0);

    ~IntDataComm();

    int SetData(SHORTI* pData, int Len, int Ident=0);

    int Size(){return m_DataLength;}

    int Send();            // This needs to be common to both sides apples = apples

    int ReceiveAll();       // gets the whole thing when it is known

    int Receive();

#ifdef MY_ARDUINO_BUILD
    void Print();
#else
    void Print(FILE* pFile=NULL);
#endif
    
};
//
// This small timer class is used to time operations in millisecond
// it works the same way as matlab tick-tock timer, unlike the other
// comms data classes it does include some basic functionality
//
// It may be used in a variety of different ways to time 
// function calls and comms processes or both sequentially
// It can also be used to time round trip times between processors

class TimerComm : public DataComm
{
    MYDWORD         m_Tick;     // set to the start record time in msec
    MYDWORD         m_Tock;     // returns delta time since last tick or -1 if tick not set
    long int        m_Sign;
#ifndef MY_ARDUINO_BUILD
    struct timeval  m_Start;    // used by MyMillis()
    struct timeval  m_Now;      // used by MyMillis()
#endif

public:
    
    TimerComm(int ID=0);

    void SetTick(MYDWORD Tick){m_Tick = Tick;}
    MYDWORD GetTick(){return m_Tick;}

    int Size(){return 2;}

#ifndef MY_ARDUINO_BUILD
    void    StartTimer();    // resets the timer m_Start
    MYDWORD   MyMillis();    // returns milliseconds elapsed since StartTimer() 
    MYDWORD   MyMicros();    // returns microseconds elapsed since  StartTimer()
#endif
    long int Sign(){return m_Sign;}

    virtual void Tick();     // start the timer

    virtual int  Tock();     // returns the time delta since Tick() will be -1 if Tick not previously set

    virtual int  Send();     // send the timer object over the comms link

    virtual int  Receive();  // receive a timer object

    virtual int  Return();
    
    void MuTick();

    int MuTock();
};

//
// FloatDataComm is a generic transport for vectors of floats 
// to allow application-specific sub-classing an identity
// is set, note that the integers are 32-bit on all machines
//
class FloatDataComm : public DataComm
{
    SHORTI      m_DataLength;      // The number of data items
    float*      m_pVec;            // The data vector we dont own it 
                                   // so it must be supplied and of valid length

public:

    FloatDataComm(float* pVec=0, int Len=0, int Ident=0); 

    ~FloatDataComm();

    int SetData(float* pVec, int Len, int Ident=0);  // set the data this is not a copy

    int Size(){return m_DataLength;}

    float* GetData(){return m_pVec;}          // return pointer to data

    float GetData(int i){
        return m_pVec[i];
    }

    int Send();            // These needs to be common to both sides apples = apples

    int Receive();
    
    int ReceiveAll();
    
    void Print(FILE* pFile=NULL);
};

//
// DepthDataComm is a transport layer for depth camera image data  
// it is recorded as unsigned short int, the camera fixes 
// datalength. 
//
// It deviates from the behaviour of the other DataComm derived types
// in that it does not use the base DataComm::Send() and Receive() functionality
// Also it is not present in the Arduino versions of this code as it is
// only intended for transmission Linux-to-Linux 
//

class DepthDataComm : public DataComm
{
    MYDWORD     m_DataLength;      // Length in bytes of dataload
    MYWORD*     m_pVec;            // The data vector we dont own it 
                                   // so it must be supplied and of valid length
    MYDWORD     m_LongByteLen;     // this class cannot use the base class m_ByteLen

public:

    DepthDataComm(MYWORD* pVec=0, MYDWORD Len=0); 

    ~DepthDataComm();

    int SetData(MYWORD* pVec, MYDWORD Len);  // set the data this is not a copy

    int Size(){return (int)m_DataLength;}

    MYWORD* GetData(){return m_pVec;}          // return pointer to data

    int Send();            // These needs to be common to both sides apples = apples

    int ReceiveAll();

    int Receive();
    
};

//
// The SeqCmd Class is defined to implement sequenced 
// commands. The principle being that each command is 
// directed to the specific channel on the given device. 
// The next command in the sequence is not executed
// until the previous command has returned from the 
// device on which channel the command was requested
// This method could be extended to include parallel
// task sequences for walking robots.
//
// It is derived from DataComm to simplfy
// Communications and Send and Receive 
//
class RobSeqCmd : public DataComm
{
    static SHORTI  m_SeqIndex;
    int            m_Active;

public:
    SHORTI     m_SeqNo;
    SHORTI     Device;
    SHORTI     Channel;
    SHORTI     Step;

    RobSeqCmd();

    void Reset(){
        m_SeqNo=0;
        m_Active=0;
        Device=0;
        Channel=0;
        Step=0;
    }


    int Active(){return m_Active;}

    void Done(){ m_Active = 0;}

    int Next()
    {
        if(m_Active){
            return 0;   // cannot do a new action until the last one is done
        }
        m_SeqIndex++;
        m_SeqNo=m_SeqIndex;
        m_Active=1;
        return m_SeqNo;
    }

    int FromString(const char* pData);

    int FromFile(FILE* pFile);

    int Me(){return m_SeqNo;}

    int IsMe(int Seq){
        return (Seq==m_SeqNo);
    }

    int Check(RobSeqCmd& Return);

    int Send();            // These needs to be common to both sides apples = apples

    int Receive();

    void Print();
    
};

class RobRTCmd : public DataComm
{

protected:
    SHORTI     m_Device;
    SHORTI     m_Chan;
    SHORTI     m_Amp;
    SHORTI     m_Vel;

public:

    RobRTCmd();

    int FromString(const char* pData);

    int Send();            // These needs to be common to both sides apples = apples

    int Receive();

    void Print();
};

// prevents compiler warnings on unused variable
// that are kept for debug
inline int Unused(int c)
{
    if(c){
        return 1;
    }
    return 0;
}

// uncomment the following line to activate debug code
// #define MY_DEBUG

void PrintDecodeString(MYBYTE* pData, int Len);
//
// some common Comms types used either end
// we declare them on the frame though most of them 
// have dynamic memory stored and managed internally 
// They are not threadPrintBuf safe and should only be used in one
// process (the one doing the communications)

// CommIn is used to read in the header for all following types
extern CommType CommIn;

extern Command  CmdIn;
extern Command  CmdOut;

extern CommString TextIn;
extern CommString TextOut;


extern DepthDataComm DepthOut;


//
// commonly used definitions
//
const char Colon = ':';
const char exc = '!';
const char CR = '\r';
const char LF = '\n';
const char EndCh = LF;
const char EndLn = LF;

//
// Memory on the Arduino is limited so these static buffers
// need to be kept small however wisdom of using the same buffer
// for all classes may prove problematic at some point
// there is a potential danger if asynchronous processes overlap
// that the buffer in use with one process is overwritten by another
// so they are definitely not multi-thread safe
//

#define KEY_BUF_LIM     80
#define MAX_TEXT_STRING 256

extern char KeyBuf[KEY_BUF_LIM];
extern char ErrorBuf[MAX_TEXT_STRING+KEY_BUF_LIM];
extern char PrintBuf[MAX_TEXT_STRING];
extern char TextBuf[MAX_TEXT_STRING];
extern char SeqBuf[MAX_TEXT_STRING];
extern char MesgBuf[MAX_TEXT_STRING];
//
// Flags and definitions used in messaging
//
extern const char* MsgSrc;

#define IP_PORT 8080

extern int HaveScreen;     // output to local console
extern int HavePipe;       // input/output to carrage
extern int HaveWIFI;       // input/output to track
extern int HaveDump;       // output to local dump file

extern int Msg_Pipe_FD;
extern int Msg_WIFI_FD;

void PrintM(const char* pMess);

#endif  // COMMTYPES_H
