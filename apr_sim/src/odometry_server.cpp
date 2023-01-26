#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>

#define MAXPENDING 5
#define LOSSPERCENT 0

void DieWithError(std::string errorMessage)
{
    std::cout << errorMessage << std::endl;
    exit(1);
}

class OdometrySub{
    public:
        OdometrySub(){
            odom_sub = nh.subscribe("/odom", 1, &OdometrySub::odom_cb, this);

            if ((serverSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
                std::cout << "Odometry: socket() failed" << std::endl;
            
            memset(&echoServAddr, 0, sizeof(echoServAddr));
            echoServAddr.sin_family = AF_INET;
            echoServAddr.sin_addr.s_addr = htonl(INADDR_ANY);
            echoServAddr.sin_port = htons(echoServPort);

            if (bind(serverSocket, (sockaddr*) &echoServAddr, sizeof(echoServAddr)) < 0)
                std::cout << "Odometry: bind() failed" << std::endl;

            if (listen(serverSocket, MAXPENDING) < 0)
                std::cout << "Odometry: listen() failed" << std::endl;
        }

        ~OdometrySub(){
            close(clientSocket);
        }

        void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg){
            clntLen = sizeof(echoClntAddr);

            if ((clientSocket = accept(serverSocket, (sockaddr*) &echoClntAddr, &clntLen)) < 0)
                std::cout << "Odometry: accept() failed" << std::endl;

            std::cout << "Odometry: Handling client " << inet_ntoa(echoClntAddr.sin_addr) << std::endl;

            std::stringstream port_msg;


            port_msg << "---START---{\"header\": {\"seq\": " << odom_msg->header.seq << ", \"stamp\": {\"secs\": " << odom_msg->header.stamp.sec << ", \"nsecs\": " << odom_msg->header.stamp.nsec << "}, \"frame_id\": \"" << odom_msg->header.frame_id << "\"}, \"child_frame_id\": \"" << odom_msg->child_frame_id << "\", \"pose\": {\"pose\": ";
            port_msg << "{\"position\": {\"x\": " << odom_msg->pose.pose.position.x << ", \"y\": " << odom_msg->pose.pose.position.y << ", \"z\": " << odom_msg->pose.pose.position.z << "}, \"orientation\": {\"x\": " << odom_msg->pose.pose.orientation.x << ", \"y\": " << odom_msg->pose.pose.orientation.y << ", \"z\": " << odom_msg->pose.pose.orientation.z << ", \"w\": " << odom_msg->pose.pose.orientation.w << "}}, \"covariance\": [";

            for(int i = 0; i < odom_msg->pose.covariance.size(); i++){
                if(i == odom_msg->pose.covariance.size()-1){
                    port_msg << odom_msg->pose.covariance.at(i) << "]}, ";
                }else{
                    port_msg << odom_msg->pose.covariance.at(i) << ", ";
                }
            }

            port_msg << "\"twist\": {\"twist\": {\"linear\": {\"x\": " << odom_msg->twist.twist.linear.x << ", \"y\": " << odom_msg->twist.twist.linear.y << ", \"z\": " << odom_msg->twist.twist.linear.z << "}, \"angular\": {\"x\": " << odom_msg->twist.twist.angular.x << ", \"y\": " << odom_msg->twist.twist.angular.y << ", \"z\": " << odom_msg->twist.twist.angular.z << "}}, \"covariance\": [";

            for(int i = 0; i < odom_msg->twist.covariance.size(); i++){
                if(i == odom_msg->twist.covariance.size()-1){
                    if((rand() % 100) >= LOSSPERCENT)
                        port_msg << odom_msg->twist.covariance.at(i) << "]}}___END___";
                }else{
                    port_msg << odom_msg->twist.covariance.at(i) << ", ";
                }
            }


            message_string = port_msg.str();
            message = message_string.c_str();
            message_length = strlen(message);

            if (send(clientSocket, message, message_length, 0) != message_length)
                DieWithError("Odometry: send() failed");

        }

    private:
        // ROS Stuff
        ros::NodeHandle nh;
        ros::Subscriber odom_sub;

        // TCP_Stuff
        int serverSocket;
        int clientSocket;
        sockaddr_in echoServAddr;
        sockaddr_in echoClntAddr;
        unsigned short echoServPort = 9998;
        unsigned int clntLen;

        // Message decoding/encoding
        std::string message_string;
        char const* message;
        int message_length;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Odometry_Server_Node");
    OdometrySub OdometryObject;

    ros::spin();

    return 0;
}