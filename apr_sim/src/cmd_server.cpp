#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#define MAXPENDING 5
#define RCVBUFSIZE 1000

void DieWithError(std::string errorMessage)
{
    std::cout << errorMessage << std::endl;
    exit(1);
}

class CmdPub{
    public:
        CmdPub(){
            cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

            if ((serverSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
                std::cout << "socket() failed" << std::endl;
            
            memset(&echoServAddr, 0, sizeof(echoServAddr));
            echoServAddr.sin_family = AF_INET;
            echoServAddr.sin_addr.s_addr = htonl(INADDR_ANY);
            echoServAddr.sin_port = htons(echoServPort);

            if (bind(serverSocket, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
                std::cout << "bind() failed" << std::endl;

            if (listen(serverSocket, MAXPENDING) < 0)
                std::cout << "listen() failed" << std::endl;

            clntLen = sizeof(echoClntAddr);

            if ((clientSocket = accept(serverSocket, (sockaddr*) &echoClntAddr, &clntLen)) < 0)
                std::cout << "accept() failed" << std::endl;

            std::cout << "Handling client " << inet_ntoa(echoClntAddr.sin_addr) << std::endl;

            HandleTCPClient();
        }

        ~CmdPub(){}

    void HandleTCPClient()
    {
        std::string delimiter;
        geometry_msgs::Twist message;
        char echoBuffer[RCVBUFSIZE];
        int recvMsgSize;

        if ((recvMsgSize = recv(clientSocket, echoBuffer, RCVBUFSIZE, 0)) < 0)
            DieWithError("recv() failed");

        std::string message_string(echoBuffer);
        std::stringstream message_stream(message_string);

        message_stream >> delimiter >> message.linear.x >> delimiter >> message.angular.z >> delimiter;

        cmd_pub.publish(message);
        
        while (recvMsgSize > 0)
        {

            if ((recvMsgSize = recv(clientSocket, echoBuffer, RCVBUFSIZE, 0)) < 0)
                DieWithError("recv() failed");

            std::string message_string(echoBuffer);
            std::stringstream message_stream(message_string);

            message_stream >> delimiter >> message.linear.x >> delimiter >> message.angular.z >> delimiter;

            cmd_pub.publish(message);
        }

        close(clientSocket);
    }

    private:
        // ROS Stuff
        ros::NodeHandle nh;
        ros::Publisher cmd_pub;

        // TCP_Stuff
        int serverSocket;
        int clientSocket;
        struct sockaddr_in echoServAddr;
        struct sockaddr_in echoClntAddr;
        unsigned short echoServPort = 9999;
        unsigned int clntLen;

        std::string message_string;
        char const* message;
        int message_length;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Cmd_Server_Node");
    CmdPub CmdObject;

    ros::spin();

    return 0;
}