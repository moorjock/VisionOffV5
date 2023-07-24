//
// <MyUnixSerial.h>
// See Header for details
//
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>	// mkfifo
#include <sys/mman.h>	// mlock
#include <sys/ioctl.h>

#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */
#include <string.h>
#include <poll.h>


#include "MyDataTypes.h"
#include "FMat.h"
#include "MyUnixSerial.h"
#include "CommTypes.h"

#define MY_DEBUG

Serial::Serial(const char* pName)
{

	m_File = -1;
	m_Timeout=0;
	m_pPortName=NULL;
	if(pName){
		m_pPortName=pName;
	}
}

int Serial::OpenPort(const char* pPortName)
{
//
// this is the only way to open an Arduino or serial port
// it must be called before Serial::begin()
//
	if(pPortName!=NULL){
	    m_pPortName = pPortName;

	    m_File  = open(m_pPortName,O_RDWR|O_SYNC);     // |O_SYNC|O_NOCTTY);  // may be useful);	/* /dev/ttyACM0  matches the Arduino terminal name   */
 
	    if(m_File<=0){
	        PrintM("Serial::OpenPort Error");
	        return 0;
	    }
		return 1;
	} else {
		return 0;
	}
}

int Serial::OpenFile(int InOut, const char* pFname)
{
// Although this class was originally designed around serial ports
// it can also use the same low-level functionallity on disk files.
// Thus OpenFile does exactly that, however it is designed to work
// in only one direction i.e. writing or reading but not both
// based on the InOut == 1 || == 2
// OpenFile() is mutex with begin() below
//
	m_pPortName = pFname;
	if(InOut==1){
		m_File = open(m_pPortName,O_WRONLY|O_CREAT|O_TRUNC, S_IRWXU|S_IRWXG|S_IRWXO );	// 0666??
	} else if(InOut==2){
		m_File = open(m_pPortName,O_RDONLY);
	} 
	StartTimer();
	setTimeout(10);	// a convenient default
	return m_File;
}

int Serial::begin(int baud)
{
//
// This function is strictly for serial transmission over RS323 or USB
// use OpenFile above for everything else
//
// Note: this Unix version ONLY sets the default transmission 
// values to: 8 data bits, no parity, one stop bit. 
// Also note that a baud of 115200 seems to be as good as it gets
// these are unix api calls
// dont use this use OpenPort above
//    m_File = open(m_pPortName,O_RDWR);	// | O_NOCTTY | O_NDELAY);	/* /dev/ttyACM0  matches the Arduino terminal name   */
			   							/* O_RDWR Read/Write access to serial port           */
										/* O_NOCTTY - No terminal will control the process   */
										/* O_NDELAY -Non Blocking Mode,Does not care about-  */
										/* -the status of DCD line,Open() returns immediatly */                                        
    struct termios PortSet;	    // structure used to set the terminal parameters
//
// open the serial (USB) port, without which nothing
//

// Get the current attributes of this Serial port
	tcgetattr(m_File, &PortSet);	

    switch (baud)
    {
        case 9600:
     		cfsetispeed(&PortSet,B9600); /* Set Read  Speed as 9600                   */
	    	cfsetospeed(&PortSet,B9600);  /* Set Write Speed as 9600                      */
            break;
        case 19200:
     		cfsetispeed(&PortSet,B19200); 
	    	cfsetospeed(&PortSet,B19200); 
            break;
        case 38400:
     		cfsetispeed(&PortSet,B38400); 
	    	cfsetospeed(&PortSet,B38400);  
            break;
        case 57600:
     		cfsetispeed(&PortSet,B57600); 
	    	cfsetospeed(&PortSet,B57600); 
            break;
        case 115200:
     		cfsetispeed(&PortSet,B115200); 
	    	cfsetospeed(&PortSet,B115200); 
            break;
        case 230400:
     		cfsetispeed(&PortSet,B230400); 
	    	cfsetospeed(&PortSet,B230400); 
            break;
        case 576000:
     		cfsetispeed(&PortSet,B57600); 
	    	cfsetospeed(&PortSet,B57600); 
            break;
        case 1152000:
     		cfsetispeed(&PortSet,B1152000); 
	    	cfsetospeed(&PortSet,B1152000); 
            break;
        case 4000000:       // this is maximum baud according to definition in <terminos.h>
     		cfsetispeed(&PortSet,B4000000); 
	    	cfsetospeed(&PortSet,B4000000); 
            break;
        default:
     		cfsetispeed(&PortSet,B9600); // lowest common denominator
	    	cfsetospeed(&PortSet,B9600); 
            break;
    }

// these match the default settings for Arduino Serial comms
	PortSet.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
	PortSet.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	PortSet.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
	PortSet.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

    PortSet.c_cflag         &=  ~CRTSCTS;       // no flow control
    PortSet.c_cc[VMIN]      =   1;              // read doesn't block
    PortSet.c_cc[VTIME]     =   1;              // 0.1 seconds read timeout

    PortSet.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    cfmakeraw(&PortSet);

// Flush Port, then applies attributes
    tcflush( m_File, TCIFLUSH );

	if((tcsetattr(m_File,TCSANOW,&PortSet)) != 0) {
		/* Set the attributes to the termios structure*/
	    PrintM("Error Setting Serial port Attributes");
		return -1;
	} else{
		sprintf(ErrorBuf,"Started on FD = %d BaudRate = %d data bits =8  StopBits = 1 No Parity\n",m_File, baud);
        PrintM(ErrorBuf);
	}

// start the millisecond timer

	StartTimer();
	setTimeout(10);	// a convenient default
//
	return m_File;
}

void  Serial::StartTimer()
{
//
// resets m_Start
//
    gettimeofday(&m_Start, NULL);
}

unsigned long Serial::millis()
{
//
// This function replicates the functionallity of the Arduino millis()
// function which returns the elapsed time in milliseconds since the above
// StartTimer() function was called. This is slightly different to the Arduino
// implementation which starts from when the device is turned on, call behaviour 
// and usage is otherwise identical
//
	long int useconds;
	unsigned long seconds, msec;  

	gettimeofday(&m_Now,NULL);

	seconds  = m_Now.tv_sec  - m_Start.tv_sec;
    useconds = (long int)m_Now.tv_usec - (long int)m_Start.tv_usec;

    msec = (unsigned long)((seconds) * 1000 + (long int)(double(useconds)/1000.0));

	return msec;
}               // returns milliseconds elapsed time since last StartTimer() 

unsigned long Serial::micros()
{
//
// This function replicates the functionallity of the Arduino micros()
// function which returns the elapsed time in microseconds since the above
// StartTimer() function was called. This is slightly different to the Arduino
// implementation which starts from when the program is run, call behaviour 
// and usage is otherwise identical
//
	long int useconds, usec;
	unsigned long seconds;  

	gettimeofday(&m_Now,NULL);

	seconds  = m_Now.tv_sec  - m_Start.tv_sec;
    useconds = (long int)m_Now.tv_usec - (long int)m_Start.tv_usec;

    usec = useconds + (unsigned long)(seconds) * 1000000;

	return usec;
}               // returns microseconds elapsed time since last StartTimer() 


void Serial::setTimeout(int mSec)
{
//
// This is the read timeout used in the
// below available() function
//

	m_Timeout = mSec;
}

int  Serial::available(int TimeFlg)
{
//
// This version now uses poll() to check for data available on m_File
// if TimeFlg >0  blocks until m_Timeout (msec) expires or data arrives
// if TimeFlg ==0 nonblocking and will return without waiting
// if TimeFlg = -1 will block indefinitely until an even occours 
//
	struct pollfd fds;
	int ret=0, count=0;
 
	fds.fd=m_File;
	fds.events=POLLIN;
	fds.revents=0;

	if(TimeFlg>0){
	    ret = poll(&fds,1,m_Timeout);
	} else{
		ret = poll(&fds,1,TimeFlg);
	}
	
	if(ret>0){
		ioctl(m_File, FIONREAD, &count);
		return count;
	}

	if(fds.revents & POLLIN){
        ret=1;
	}
	return ret;
}

int Serial::flush()
{
// This again replicates functionallity available in Arduino
// using a linux low-level file handler fsync to ensure the
// write buffer is flushed. It blocks the process until 
// all data is written to the port
//
    return fsync(m_File);
}

int Serial::read(MYBYTE& c)
{
// reads a single char and returns it
    int n;
    n = ::read(m_File,&c,1);
    return n;
}

int Serial::readBytes(char* pbuf, int length)
{
// use this for reading text strings ONLY
// it will terminate on the first encounter of '\n'
// If count read is less than length an additional
// NULL byte is added after the terminator is
// detected.
//
	int n=0, count=0;
    char Ch = '\0';
    do{
	    n = ::read(m_File,&Ch,1);
        pbuf[count] = Ch;
        count += n;
	} while( Ch != '\n' && n > 0 && count < length);
// if the string is terminated by '\n' and it is less than length
// add a '\0' 	
	if((count<length) && Ch=='\n'){
		pbuf[count] = '\0';
	}
	return count;
}

int Serial::readBytesUntil(char EndCh, char* pBuf, int Num, int& found)
{
// use this for reading text strings ONLY
// Similar to the Arduino function of the same name
// slightly more complex than readBytes above looks for EndCh
// and returns found=1 if it is detected.
// use readBytes above if only the '\n' is being checked
//
	int n=0, count=0;
    char Ch = '\0';
        do{
		    n = ::read(m_File,&Ch,1);
			if(n<0){
            	found=-1;
            	break;
        	}
			if(Ch==EndCh){
				found=1;
				break;
			}
            pBuf[count] = Ch;
            count += n;
	    } while( count < Num);

// if the string is terminated by EndCh and it is less than Num
// add a following '\0' 	
	if((count<Num) && (Ch==EndCh) ){
		pBuf[count] = '\0';
	}

	return count;
}

int Serial::readDataBytes(MYBYTE* pbuf, int length)
{
// Use this ONLY for Binary data stream
// It will read a fixed length of byte data
// it will not look for a special terminator character
// in the data stream so it will block if count<length
//
    int n=0, count=0;
    char Ch = '\0';
    do{
		n = ::read(m_File,&Ch,1);
        pbuf[count] = (MYBYTE)Ch;
        count += n;
    } while( (n>0) && (count < length));

    return count;
}

int Serial::GetMessage(const char* pMess, int TimeFlg)
{
//
// This function is designed to intercept the required message pMess
// within a stream of data from stream pStr which can also contain
// other information messages ahead of the requires pMess
// these are identified by a "#" in the first 6 characters
// If the message is not found MesgBuf can be interogated
// to see if another message was present
//
// Returns:
//   1		if the function found pMess in the stream
//   0      pMess was not found
//          
//
// The function will print any '#' comments directly to the
// available print ports, which may redirect to pipes etc.
//
// Behaviour is modified by TimeFlg
//   -1		will block indefinitely until pMess is located
//    0		will return imediately if there is nothing in the buffer
//   x>0    will block for m_Timeout =/= x msec 
//
	int i, len, PrtFlg, test=0;
//
// while(available) will spin through any 
// incomming messages prefixed with '#'
// printing them to the available open ports
// The first message that is not prefixed
// will trip the loop and then be tested for pMess
// or a sequence return
//
	test = available(TimeFlg);
	while(test){
		PrtFlg=0;
		len = readBytes(MesgBuf,MAX_TEXT_STRING);
		len = my_min(len,6);
		for(i=0;i<len;i++){
			if(MesgBuf[i]=='#'){
				PrtFlg=1;
			}
		}
		if(PrtFlg){
			PrintM(MesgBuf);
		} else {
			break;
		}
		test = available(TimeFlg);
	}

	if(strcasestr(MesgBuf,pMess)){
		return 1;
	} else if(test>0){
// requested message not found
		sprintf(ErrorBuf,"GetMessage Unexpected MesgBuf>%s<\n",MesgBuf);
		return 0;
	}
	return 0;
}

void Serial::ForwardMessage(const char* pMess)
{
//
// using this function # messages received by GetMessage()
// above on one port can be re-directed to this port without copy
// caution needed as this is not thread safe
//
	Serial::write(pMess,strlen(pMess));
}

void Serial::SendMessage(const char* pMess)
{
// send a # text message to this port

 	sprintf(ErrorBuf,"# %s> %s\n",MsgSrc,pMess);
 	MYDWORD len = strlen(ErrorBuf);
 	Serial::write(ErrorBuf,len);
}

MYDWORD Serial::write(const void* buf, MYDWORD len) 
{
	if(len<=0){
		len = strlen((const char*)buf);
	}
    return ::write(m_File,buf,len); // return wot is writted
}

int Serial::HardStop(const char* pDev, int TimeFlag)
{
// The current strategy more or less obligatory with
// Linux is that Arduinos and other serial devices
// simply push out data at the Real-time frame rate
// This can cause problems (particularly when debugging)
// when the receiver does not ingest the messages 
// at the same rate so a backlog builds up preventing
// instantaneous halting of the outflow. There should
// be an easier way to clear the buffer but this 
// function simply purges the incomming stream until it
// is empty and can then be stopped
//
	char Stop[] = ":C  3!\n";
	write(Stop);
    if(GetMessage("STOP")){
        GetMessage(":C  3!");
    } else {
		sprintf(ErrorBuf,"Warning Hard-Stopping %s",pDev);
		PrintM(ErrorBuf);
        while(available(TimeFlag)){
            readBytes(TextBuf,MAX_TEXT_STRING);
#ifdef MY_DEBUG
			printf("%s\n",TextBuf);
#endif
		}
		GetMessage("STOP",1);
        GetMessage(":C  3!",1);		
	}
	return 1;
}
void Serial::close()
{
/* Close the Serial port */
	if(m_File>-1){
	    flush();
	    ::close(m_File);
	    m_File = -1;
	}
}

int PipeSerial::Open(int InOut)
{

	m_InOut = InOut;
	switch(m_InOut){	// set 1 for an input pipe 2 for an output
		case 1:
    		mkfifo(m_pPortName, 0666);	// 0666 == permission for everyone to read/write to the pipe
			m_File = open(m_pPortName,O_RDONLY);
		break;
		case 2:
    		mkfifo(m_pPortName, 0666);	// 0666 == permission for everyone to read/write to the pipe
			m_File = open(m_pPortName,O_WRONLY);
		break;
		default:
		 return -1;
	}

	setTimeout(10);
	return m_File;

}

int PipeSerial::available(int TimeFlg)
//
// indicates data available to read
// for this pipe version it also returns the
// number of unread bytes on the pipe =0 
// if none available
{
	int ret, count=0;

	ret=Serial::available(TimeFlg);
	if(ret>0){
		ioctl(m_File, FIONREAD, &count);
		return count;
	}
	return ret;
}

MemoryFile::MemoryFile()
{
	m_File=-1;
    m_FilePos=0;
    m_Ptr=NULL;
    m_Size=0;
}
	
    
MemoryFile::~MemoryFile()
{
    if(m_Ptr!=NULL){
		munlockall();
		free(m_Ptr);
		m_Ptr=NULL;
	}

	if(m_File>-1){
		close();
		m_File=-1;
	}
}
    
MYDWORD MemoryFile::Allocate(MYDWORD Size)
{
    if(m_File>-1){
        m_Ptr = calloc(Size+1,sizeof(MYBYTE));
        if(m_Ptr<=0){
            PrintM("MemoryFile calloc Error in Allocate");
            m_Size=0;
            return -1;
        }
        int ret = mlock(m_Ptr,(m_Size+1)*sizeof(MYBYTE));
        if(ret){
           PrintM("MemoryFile mlock Error in Allocate");
		   m_Size=0;
		   return -1;
		}
		m_Size = Size;
		m_FilePos=0;
    } else {
        m_Size=-1;
    }
    return m_Size;
}

MYDWORD MemoryFile::Initialise()
{
//	MYBYTE* pB;
	if(m_Ptr>0 and m_Size>0){
//		pB = (MYBYTE*)m_Ptr;
//		for(MYDWORD i=0;i<m_Size;i++){
//			pB[i] = 0;
//		}
		memset(m_Ptr,0,m_Size);
		return m_Size;
	}
	return -1;
}

MYDWORD MemoryFile::write(const void* buf, MYDWORD len)
{
//  write a buffer of binary data to memory
//    MYDWORD i;
    MYBYTE* pB = (MYBYTE*)buf;
    MYBYTE* pP = (MYBYTE*)m_Ptr + m_FilePos*sizeof(MYBYTE);
    if((m_FilePos+len) > m_Size){
		PrintM(" MemoryFile Overrun in write");
		return -1;
    }

	memcpy(pP,pB,len);

//    for(i=m_FilePos;i<len;i++){
//		pP[i] = pB[i];
//    }

    m_FilePos+=len;
    return len;
}

size_t MemoryFile::CommitToDisk()
{
	MYBYTE* pP;
    if((m_File>-1) && (m_Ptr>0) && (m_Size>0)){
    	 pP = (MYBYTE*)m_Ptr + m_FilePos*sizeof(MYBYTE);
		*pP = EOF;
		m_FilePos++;
		size_t lret = ::write(m_File,m_Ptr,m_FilePos);
		flush();
		int ret = munlock(m_Ptr, m_Size+1);
		if(ret){
 	       PrintM(" MemoryFile munlock Error ");
		}

		free(m_Ptr);
		m_Ptr=NULL;
		m_Size=0;
		return lret;
    }
    return -1;
}

