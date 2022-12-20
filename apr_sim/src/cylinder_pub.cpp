#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define CYLINDER_X 0.58
#define CYLINDER_Y -0.195
#define CYLINDER_HEIGHT 1.0
#define CYLINDER_RADIUS 0.034

int main(int argc, char** argv){
    
    ros::init(argc, argv, "cylinder_pub_node");

    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<visualization_msgs::Marker>("/apr/map/cylinder", 1, true);

    visualization_msgs::Marker cylinder;

    cylinder.header.frame_id = "map";
    cylinder.id = 0;
    cylinder.type = visualization_msgs::Marker::CYLINDER;
    cylinder.pose.position.x = CYLINDER_X;
    cylinder.pose.position.y = CYLINDER_Y;
    cylinder.pose.position.z = CYLINDER_HEIGHT / 2;
    cylinder.pose.orientation.x = 0.0;
    cylinder.pose.orientation.y = 0.0;
    cylinder.pose.orientation.z = 0.0;
    cylinder.pose.orientation.w = 1.0;
    cylinder.scale.x = CYLINDER_RADIUS * 2;
    cylinder.scale.y = CYLINDER_RADIUS * 2;
    cylinder.scale.z = 1.0;
    cylinder.color.r = 0.0;
    cylinder.color.g = 0.0;
    cylinder.color.b = 1.0;
    cylinder.color.a = 1.0;

    map_pub.publish(cylinder);

    while(ros::ok()){
        ros::spin();
    }

    return 0;
}
