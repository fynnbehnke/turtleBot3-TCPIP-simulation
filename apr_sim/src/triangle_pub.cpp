#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define TRIANGLE_X 0.989
#define TRIANGLE_Y -0.463
#define TRIANGLE_LENGTH_SIDE 0.3
#define TRIANGLE_LENGTH_BOT 0.286

int main(int argc, char** argv){
    
    ros::init(argc, argv, "triangle_pub_node");

    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<visualization_msgs::Marker>("/apr/map/triangle", 1, true);

    geometry_msgs::Point triangle_point;
    visualization_msgs::Marker triangle;

    triangle.header.frame_id = "map";
    triangle.id = 4;
    triangle.type = visualization_msgs::Marker::LINE_STRIP;
    
    triangle_point.x = TRIANGLE_X;
    triangle_point.y = TRIANGLE_Y;
    triangle_point.z = 0;   
    triangle.points.push_back(triangle_point);

    triangle_point.x = TRIANGLE_X - TRIANGLE_LENGTH_BOT / 2;
    triangle_point.y = TRIANGLE_Y - TRIANGLE_LENGTH_SIDE;
    triangle_point.z = 0;   
    triangle.points.push_back(triangle_point);

    triangle_point.x = TRIANGLE_X + TRIANGLE_LENGTH_BOT / 2;
    triangle_point.y = TRIANGLE_Y - TRIANGLE_LENGTH_SIDE;
    triangle_point.z = 0;   
    triangle.points.push_back(triangle_point);

    triangle_point.x = TRIANGLE_X;
    triangle_point.y = TRIANGLE_Y;
    triangle_point.z = 0;   
    triangle.points.push_back(triangle_point);
    
    triangle.scale.x = 0.015;
    triangle.color.r = 1.0;
    triangle.color.g = 0.0;
    triangle.color.b = 0.0;
    triangle.color.a = 1.0;

    map_pub.publish(triangle);

    while(ros::ok()){
        ros::spin();
    }

    return 0;
}
