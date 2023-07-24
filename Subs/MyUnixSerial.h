//
// <MyUnixSerial.h>
//
// This class provide a simple Arduino-like serial comms interface to linux 
// functions the main objective being to provide a common interface for the
// rest of the codes, some classes of which are shared.
//
// MyUnixSerial.h/cpp       This class provides the Arduino-like serial interface
// MyDataTypes.h/cpp        Defines allowable datatypes for comms
// ComTypes.h/cpp           Used on all sides (with some compiler-directed modifications)
//                          contains definitions of the comms data classes
//
// An important lesson is that when working with asynchronous codes
// calls to sleep(), usleep() and select()/poll() (when called with a timeout)
// are invariably a bad idea since they automatically yield control to the 
// process scheduler.
//
// This would not be such an issue were it not for the fact that vanilla
// Linux has a minimum scheduling resolution of 10msec. A workaround is to have 
// Arduino drive the realtime clock and keep other linux boxes locked into 
// read mode. Other methods such as spinning in place don't work because 
// modern processors are thermally optimised and use the scheduller to 
// control thermal overload
//-----------------------------------------------------------------------------------
//  Version:       2.0
//  Changes:       Various changes PrintM, HardStop and MemoryFile class added
//  Date:          5 May 2021 
//  Programmer:    A. Shepherd
//-----------------------------------------------------------------------------------
//  Version:       1.0
//  Changes:       First version developed on Linux Laptop
//  Date:          01 October 2019 
//  Programmer:    A. Shepherd
//-----------------------------------------------------------------------------------
//
#ifndef MY_UNIX_SERIAL
#define MY_UNIX_SERIAL

class Serial
{
protected:

    const char*     m_pPortName;    // the linux device name i.e. /dev/ttyACMX where X may be 0-3
    struct timeval  m_Start;        // used by millis()
    struct timeval  m_Now;          // used by millis()
    int             m_Timeout;      // in msec used by available() for device timeout

    int             m_File;         // the low-level file descriptor FD

public:

    int GetFD(){return m_File;}
    
    Serial(const char* pName=NULL);     

    int OpenPort(const char* pPortName); // Normal route to setup the port

    int begin(int baud = 115200);      // Baud rate 115200 is the default for RasPi and works with Arduino
    
    virtual int OpenFile(int InOut, const char* pFname);    // InOut =1 Write =2 Read
    
    void setTimeout(int TimeOut = 1);  // timeout in msec for use with available()

    virtual int available(int TimeFlg=0);     // indicates data available to read

    void  StartTimer();                // resets m_Start

    unsigned long millis();            // returns milliseconds elapsed time since last StartTimer() 
                                       // operates like the Arduino function of the same name
    unsigned long micros();

    virtual int read(MYBYTE& c);               // read a single character

    virtual int readBytes(char* pbuf, int length);  // use for short reads only it will block

    int GetMessage(const char* pMess, int Timeout=-1);  // look for a specific message on this port
    
    void SendMessage(const char* pMess);    // send a # text message to this port

    void ForwardMessage(const char* pMess);  // forwards a # message received by GetMessage() above

    virtual int readDataBytes(MYBYTE* pbuf, int length);  // a fixed length of byte data without seeking for '\n'

    int readBytesUntil(char Endch, char* pBuf, int Num, int& found);   // read Num bytes or until EndC detected

    virtual MYDWORD write(const void* buf, MYDWORD len=0); //  write a buffer of binary data

    int HardStop(const char* pDev=NULL, int TimeFlag=0);    // purges an incomming stream until it stops

    int flush();                      // waits for pending writes to complete 
                                      // use with care it locks the process

    virtual void close();             // close the port

};
//
// This is a Linux FIFO or "Named Pipe"
// It is always better to have two pipes one for read and one for write
// not forgetting to reverse the roles at the other end of the pipes
// Testing of single pipes shows that there are problems if both ends
// can read/write to the same pipe
//
#define IS_IN_PIPE 1
#define IS_OUT_PIPE 2

class PipeSerial : public Serial
{
    int     m_InOut;       // // set 1 for an input pipe 2 for an output
public:

    PipeSerial(const char* pPort=NULL){
        m_pPortName = pPort;
    }

    int IsInPipe(){return (m_InOut==IS_IN_PIPE);}

    int IsOutPipe(){ return (m_InOut==IS_OUT_PIPE);}

    int Open(int InOut=IS_IN_PIPE);     

    int available(int TimeFlg=0);     // indicates data available to read

};

//
// This class is designed to write data to memory during a real-time process
// to prevent the process from stalling while attempting to perform disk
// write operations.
// The memory allocation must be big enough for the recording operation 
// undertaken to completion.
// Attempts to write beyond the allocation will simply fail returning -1
// Only data actually committed to memory will be saved to the file.
//
// The file must be opened _Before_ Allocation or the process will fail
// 
//
class MemoryFile : public Serial
{
    MYDWORD     m_FilePos;
    MYDWORD     m_Size;
    void*       m_Ptr;
    
public:
    MemoryFile();
    
    ~MemoryFile();
   
    MYDWORD Allocate(MYDWORD Size);

    MYDWORD Initialise();
    
    int read(MYBYTE& c){return -1;}               // not impemented

    int readBytes(char* pbuf, int length){ return -1;}  // not impemented 
            
    int readDataBytes(char* pbuf, int length){ return -1;}  // not impemented

    int readBytesUntil(char Endch, char* pBuf, int Num, int& found){ return -1;}   // not impemented

    MYDWORD write(const void* buf, MYDWORD len=0); //  write a buffer of binary data to memory
   
    size_t CommitToDisk();          // commit data to disk
    
    void close(){ Serial::close();}

};

#endif  // MY_UNIX_SERIAL


