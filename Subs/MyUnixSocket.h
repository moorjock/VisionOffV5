//
// <MyUnixSocket.h>
//
// Class to provide TCP/IP socket interface  
//
// The class provides both client and server functionallity 
//-----------------------------------------------------------------------------------
//  Version:     2.0
//  Changes:     Return fails Indexed, PrintM added
//  Date:        9 June 2021
//  Programmer:  A. Shepherd
//-----------------------------------------------------------------------------------
//  Version:     1.0
//  Changes:     First version developed on Linux Laptop
//  Date:        01 October 2019 
//  Programmer:  A. Shepherd
//-----------------------------------------------------------------------------------

#ifndef MY_UNIX_SOCKET
#define MY_UNIX_SOCKET

class MySocket : public Serial
{
    int m_ServerFD;                 // Used by the server to start comms
    int m_Server;                   // =1 if server 0 if client
    int m_Port;                     // the socket port
public:
    MySocket();

    int SetClient(const char* pIPName, int Port);

    int SetServer(int Port);

    void close();

};


// port 8080 is defined as an alternate http


#endif  // MY_UNIX_SOCKET



