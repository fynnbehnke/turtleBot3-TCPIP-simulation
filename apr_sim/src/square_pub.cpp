#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define SQUARE_X 0.474
#define SQUARE_Y -0.455
#define SQUARE_LENGTH 0.3

int main(int argc, char** argv){
    
    ros::init(argc, argv, "square_pub_node");

    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<visualization_msgs::Marker>("/apr/map/square", 1, true);

    geometry_msgs::Point square_point;
    visualization_msgs::Marker square;

    square.header.frame_id = "map";
    square.id = 3;
    square.type = visualization_msgs::Marker::LINE_STRIP;
    
    square_point.x = SQUARE_X;
    square_point.y = SQUARE_Y;
    square_point.z = 0;   
    square.points.push_back(square_point);

    square_point.x = SQUARE_X;
    square_point.y = SQUARE_Y - SQUARE_LENGTH;
    square_point.z = 0;   
    square.points.push_back(square_point);

    square_point.x = SQUARE_X + SQUARE_LENGTH;
    square_point.y = SQUARE_Y - SQUARE_LENGTH;
    square_point.z = 0;   
    square.points.push_back(square_point);

    square_point.x = SQUARE_X + SQUARE_LENGTH;
    square_point.y = SQUARE_Y;
    square_point.z = 0;   
    square.points.push_back(square_point);

    square_point.x = SQUARE_X;
    square_point.y = SQUARE_Y;
    square_point.z = 0;   
    square.points.push_back(square_point);
    
    square.scale.x = 0.015;
    square.color.r = 1.0;
    square.color.g = 0.0;
    square.color.b = 0.0;
    square.color.a = 1.0;

    map_pub.publish(square);

    while(ros::ok()){
        ros::spin();
    }

    return 0;
}
