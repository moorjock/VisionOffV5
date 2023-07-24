//
// <MyUnixSocket.cpp>
//
// See Header for details
//

#include <unistd.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <sys/socket.h> // main TCP/IP api contained in /usr/x86_64-linux-gnu/sys
#include <arpa/inet.h>	// added by me for inet_pton() contains text to binary conversion routines
#include <netinet/in.h> // contains protocol and PORT definitions, ports below 1024 are reserved


#include "MyDataTypes.h"
#include "FMat.h"
#include "MyUnixSerial.h"

#include "CommTypes.h"
#include "MyUnixSocket.h"


MySocket::MySocket()
{
    m_ServerFD=0;         // Used by the server to start comms
    m_Server=0;           // =1 if server
	m_Port=0;			  // Typically 8080 but others will also work
}

int MySocket::SetClient(const char* pIPName, int Port)
{
//
// Start a Client socket connection with the named server
//
	struct sockaddr_in serv_addr; 
	const char* hello = "Hello from Client"; 
    int valread;
    char bufIn[MAX_TEXT_STRING] = {0}; 

    m_File = socket(AF_INET, SOCK_STREAM, 0);

	if ( m_File < 0) { 
//  		PrintM("Client Socket Creation Error");
		return -1; 
	} 

	m_Port = Port;

	memset(&serv_addr, '0', sizeof(serv_addr)); 

	serv_addr.sin_family = AF_INET; 
	serv_addr.sin_port = htons(m_Port); 
//
// Convert IPv4 and IPv6 addresses from text to binary form
//
	if(inet_pton(AF_INET, pIPName, &serv_addr.sin_addr)<=0) { 
    	PrintM(" Invalid Client IP Address "); 
		return -2; 
	} 

	if (connect(m_File, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) { 
	    PrintM(" Client Connection Failed Check the Server is Running");
		return -3;
    }
//
// Do a simple send-recv to check the channel is open
//
	send(m_File , hello , strlen(hello) , 0 );
    PrintM(" Client sent Hello");

	valread = recv(m_File , bufIn, MAX_TEXT_STRING, 0); 
    sprintf(ErrorBuf,"Client Received %s\n",bufIn);
    PrintM(ErrorBuf);
		
	fcntl(m_File, F_SETFL, O_NONBLOCK);

    return valread;
}
//
//-----------------------------------------------------------------------------------
//
int MySocket::SetServer(int Port)
{
//
// At this point we don't yet have a connection to anyone
// so we set up a listening port awaiting an input
// 
    struct sockaddr_in address;
    int addrlen = sizeof(address); 
    char buffer[MAX_TEXT_STRING] = {0}; 
    int opt = 1;
    const char* hello = "Hello from server\n"; 
    int valread;
//
    m_Server=1;
	m_Port = Port;
	
// Creating socket file descriptor 

    if ((m_ServerFD = socket(AF_INET, SOCK_STREAM, 0)) <= 0) 
    { 
        PrintM("Server Socket Failed");
        return -1;
    } 
       
    // Set the socket options 
    if (setsockopt(m_ServerFD, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, 
                                                  &opt, sizeof(opt))) 
    { 
        PrintM("Server setsockopt Failed");
        return -2; 
    }

    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( m_Port ); 
       
    if (bind(m_ServerFD, (struct sockaddr *)&address,  
                                 sizeof(address))<0) 
    { 
        PrintM("Server bind Failed");
        return -3; 
    }

    if (listen(m_ServerFD, 1) < 0) 
    { 
        PrintM("Server listen Failed");
        return -4;
    }
    sprintf(ErrorBuf,"# %s> Server is awaiting a connection on port %d\n",MsgSrc,m_Port);
    PrintM(ErrorBuf);
//
// This should block until m_File is open at which point we can communicate
//
    if ((m_File = accept(m_ServerFD, (struct sockaddr *)&address,  
                       (socklen_t*)&addrlen))<0) 
    { 
        PrintM("Server accept Failed");
        return -5; 
    } else {
        PrintM("Server Connection Accepted");
    }

// receive a handshake message    
    valread = recv( m_File , buffer, MAX_TEXT_STRING, 0); 
    if(valread<0){
// error receiving handshake?
        return -6;
    }
// send back a similar handshake message
    send(m_File , hello , strlen(hello) , 0 ); 

// return number of bytes read for a sucessful connection
    return valread;
}

void MySocket::close()
{
    ::close(m_File);
}

