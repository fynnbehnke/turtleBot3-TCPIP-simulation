#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define BORDER_X 1.189
#define BORDER_Y 0.841

int main(int argc, char** argv){
    
    ros::init(argc, argv, "border_pub_node");

    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<visualization_msgs::Marker>("/apr/map/border", 1, true);

    geometry_msgs::Point    border_point;
    visualization_msgs::Marker  border;

    border.header.frame_id = "map";
    border.id = 5;
    border.type = visualization_msgs::Marker::LINE_STRIP;
    
    border_point.x = 0;
    border_point.y = 0;
    border_point.z = 0;   
    border.points.push_back(border_point);

    border_point.x =  0;
    border_point.y =  -BORDER_Y;
    border_point.z = 0;   
    border.points.push_back(border_point);

    border_point.x = BORDER_X;
    border_point.y = -BORDER_Y;
    border_point.z = 0;   
    border.points.push_back(border_point);

    border_point.x = BORDER_X;
    border_point.y = 0;
    border_point.z = 0;   
    border.points.push_back(border_point);

    border_point.x = 0;
    border_point.y = 0;
    border_point.z = 0;   
    border.points.push_back(border_point);
    
    border.scale.x = 0.015;
    border.color.r = 0.0;
    border.color.g = 0.0;
    border.color.b = 0.0;
    border.color.a = 1.0;

    map_pub.publish(border);

    while(ros::ok()){
        ros::spin();
    }

    return 0;
}
