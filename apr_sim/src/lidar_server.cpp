#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>
#include <bits/stdc++.h>

#define MAXPENDING 5

void DieWithError(std::string errorMessage)
{
    std::cout << errorMessage << std::endl;
    exit(1);
}

class LidarSub{
    public:
        LidarSub(){
            lidar_sub = nh.subscribe("/scan", 1, &LidarSub::scan_cb, this);

            if ((serverSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
                std::cout << "LiDAR: socket() failed" << std::endl;
            
            memset(&echoServAddr, 0, sizeof(echoServAddr));
            echoServAddr.sin_family = AF_INET;
            echoServAddr.sin_addr.s_addr = htonl(INADDR_ANY);
            echoServAddr.sin_port = htons(echoServPort);

            if (bind(serverSocket, (sockaddr*) &echoServAddr, sizeof(echoServAddr)) < 0)
                std::cout << "LiDAR: bind() failed" << std::endl;

            if (listen(serverSocket, MAXPENDING) < 0)
                std::cout << "LiDAR: listen() failed" << std::endl;

            clntLen = sizeof(echoClntAddr);

            if ((clientSocket = accept(serverSocket, (sockaddr*) &echoClntAddr, &clntLen)) < 0)
                std::cout << "LiDAR: accept() failed" << std::endl;

            std::cout << "LiDAR: Handling client " << inet_ntoa(echoClntAddr.sin_addr) << std::endl;
        }

        ~LidarSub(){
            close(clientSocket);
        }

        void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
            std::stringstream port_msg;


            port_msg << "---START---{\"header\": {\"seq\": " << scan_msg->header.seq << ", \"stamp\": {\"secs\": " << scan_msg->header.stamp.sec << ", \"nsecs\": " << scan_msg->header.stamp.nsec << "}, \"frame_id\": \"" << scan_msg->header.frame_id << "\"}, ";
            port_msg << "\"angle_min\": " << scan_msg->angle_min << ", \"angle_max\": " << scan_msg->angle_max << ", \"angle_increment\": " << scan_msg->angle_increment << ", \"time_increment\": " << scan_msg->time_increment << ", \"scan_time\": " << scan_msg->scan_time << ", \"range_min\": " << scan_msg->range_min << ", \"range_max\": " << scan_msg->range_max << ", \"ranges\": [";

            for(int i = 0; i < scan_msg->ranges.size(); i++){
                if(isinf(scan_msg->ranges.at(i))){
                    if(i == scan_msg->ranges.size()-1){
                        port_msg << 0.0 << "], \"intensities\": [";
                    }else{
                        port_msg << 0.0 << ", ";
                    }
                }else{
                    if(i == scan_msg->ranges.size()-1){
                        port_msg << scan_msg->ranges.at(i) << "], \"intensities\": [";
                    }else{
                        port_msg << scan_msg->ranges.at(i) << ", ";
                    }
                }
            }

            for(int i = 0; i < scan_msg->intensities.size(); i++){
                if(i == scan_msg->intensities.size()-1){
                    port_msg << scan_msg->intensities.at(i) << "]}___END___";
                }else{
                    port_msg << scan_msg->intensities.at(i) << ", ";
                }
            }

            message_string = port_msg.str();
            message = message_string.c_str();
            message_length = strlen(message);

            if (send(clientSocket, message, message_length, 0) != message_length)
                DieWithError("LiDAR: send() failed");

        }

    private:
        // ROS Stuff
        ros::NodeHandle nh;
        ros::Subscriber lidar_sub;

        // TCP_Stuff
        int serverSocket;
        int clientSocket;
        sockaddr_in echoServAddr;
        sockaddr_in echoClntAddr;
        unsigned short echoServPort = 9997;
        unsigned int clntLen;

        std::string message_string;
        char const* message;
        int message_length;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Lidar_Server_Node");
    LidarSub LidarObject;

    ros::spin();

    return 0;
}
