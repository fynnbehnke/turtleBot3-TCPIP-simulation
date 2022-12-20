// create a TCP/echo_server (in C++) on the turtlebot and test it with port 10000 TCP/echo_client (in C++) on your PC
#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket(), bind(), and connect() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_ntoa() */
#include <string.h>
#include <unistd.h>     
#include <iostream>

#define RCVBUFSIZE 32   /* Size of receive buffer */
#define MAXPENDING 5    /* Maximum outstanding connection requests */

// compile with:    gcc tcp_echo_server.cpp -lpthread -o server
// run with :       ./server 10000

/* Error handling function */
void DieWithError(std::string errorMessage)
{
    std::cout << errorMessage << std::endl;
    exit(1);
}

void HandleTCPClient(int clntSocket)
{
    char const* echoBuffer = "---START---{\"header\": {\"seq\": 11809, \"stamp\": {\"secs\": 1667556711, \"nsecs\": 341204004}, \"frame_id\": \"odom\"}, \"child_frame_id\": \"base_footprint\", \"pose\": {\"pose\": {\"position\": {\"x\": 1.9796332120895386, \"y\": -0.32911765575408936, \"z\": 0.0}, \"orientation\": {\"x\": 0.0, \"y\": 0.0, \"z\": -0.09343531727790833, \"w\": 0.995625376701355}}, \"covariance\": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, \"twist\": {\"twist\": {\"linear\": {\"x\": 0.0, \"y\": 0.0, \"z\": 0.0}, \"angular\": {\"x\": 0.0, \"y\": 0.0, \"z\": -0.002709658583626151}}, \"covariance\": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}___END___";        /* Buffer for echo string */
    int receiveMessageSize = strlen(echoBuffer);
    int counter = 0;                    /* Size of received message */

    /* Receive message from client */
    //if ((receiveMessageSize = recv(clntSocket, echoBuffer, RCVBUFSIZE, 0)) < 0)
    //    DieWithError("recv() failed");

    /* Send received string and receive again until end of transmission */
    while (counter < 10)      /* zero indicates end of transmission */
    {
        std::cout << echoBuffer << std::endl;
        /* Echo message back to client */
        if (send(clntSocket, echoBuffer, receiveMessageSize, 0) != receiveMessageSize)
            DieWithError("send() failed");

        /* See if there is more data to receive */
        //if ((receiveMessageSize = recv(clntSocket, &echoBuffer, RCVBUFSIZE, 0)) < 0)
        //    DieWithError("recv() failed");

        counter++;
        sleep(1);
    }

    close(clntSocket);    /* Close client socket */
}


int main(int argc, char *argv[])
{
    int serverSocket;                       /* Socket descriptor for server */
    int clientSocket;                       /* Socket descriptor for client */
    struct sockaddr_in echoServerAddress;   /* Local address */
    struct sockaddr_in echoClientAddress;   /* Client address */
    unsigned short echoServerPort;          /* Server port */
    unsigned int clientLength;              /* Length of client address data structure */

    if (argc != 2)     /* Test for correct number of arguments */
    {
        std::cout << "Usage:  " << argv[0] << " <Server Port>" << std::endl;;
        exit(1);
    }

    echoServerPort = std::stoi(argv[1]);  /* First arg:  local port */

    /* Create socket for incoming connections */
    if ((serverSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        std::cout << "socket() failed" << std::endl;
      
    /* Construct local address structure */
    memset(&echoServerAddress, 0, sizeof(echoServerAddress));   /* Zero out structure */
    echoServerAddress.sin_family = AF_INET;                     /* Internet address family */
    echoServerAddress.sin_addr.s_addr = htonl(INADDR_ANY);      /* Any incoming interface */
    echoServerAddress.sin_port = htons(echoServerPort);         /* Local port */

    /* Bind to the local address */
    if (bind(serverSocket, (sockaddr *) &echoServerAddress, sizeof(echoServerAddress)) < 0)
        std::cout << "bind() failed" << std::endl;

    
    /* Listening for a client to connect. */
    if (listen(serverSocket, MAXPENDING) < 0)
        std::cout << "listen() failed" << std::endl;

    /* Waiting for a client to connect. */
    while(1){
        /* Set the size of the in-out parameter */
        clientLength = sizeof(echoClientAddress);

        /* Wait for a client to connect */
        if ((clientSocket = accept(serverSocket, (sockaddr *) &echoClientAddress, &clientLength)) < 0)
            std::cout << "accept() failed" << std::endl;

        /* clntSock is connected to a client! */

        /* It prints the IP address of the client. */
        std::cout << "Handling client " << inet_ntoa(echoClientAddress.sin_addr) << std::endl;

        HandleTCPClient(clientSocket);
    }
}