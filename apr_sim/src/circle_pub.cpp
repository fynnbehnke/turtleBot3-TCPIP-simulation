#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>

#define CIRCLE_X 0.237
#define CIRCLE_Y -0.604
#define CIRCLE_RADIUS 0.15

int main(int argc, char** argv){
    
    ros::init(argc, argv, "circle_pub_node");

    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<visualization_msgs::Marker>("/apr/map/circle", 1, true);

    geometry_msgs::Point circle_point;
    visualization_msgs::Marker circle;

    circle.header.frame_id = "map";
    circle.id = 2;
    circle.type = visualization_msgs::Marker::LINE_STRIP;
    
    for(int i = 0; i <= 360; i++){
        circle_point.x = CIRCLE_X + CIRCLE_RADIUS * cos(i * (M_PI/180));
        circle_point.y = CIRCLE_Y + CIRCLE_RADIUS * sin(i * (M_PI/180));
        circle_point.z = 0;   
        circle.points.push_back(circle_point);
    }

    circle.scale.x = 0.015;
    circle.color.r = 1.0;
    circle.color.g = 0.0;
    circle.color.b = 0.0;
    circle.color.a = 1.0;

    map_pub.publish(circle);

    while(ros::ok()){
        ros::spin();
    }

    return 0;
}
