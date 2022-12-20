#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define LINE_X 0.081
#define LINE_Y -0.436
#define LINE_LENGTH 1.043

int main(int argc, char** argv){
    
    ros::init(argc, argv, "line_pub_node");

    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<visualization_msgs::Marker>("/apr/map/line", 1, true);

    geometry_msgs::Point line_point;
    visualization_msgs::Marker line;

    line.header.frame_id = "map";
    line.id = 1;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    
    line_point.x = LINE_X;
    line_point.y = LINE_Y;
    line_point.z = 0;   
    line.points.push_back(line_point);

    line_point.x = LINE_X + LINE_LENGTH;
    line_point.y = LINE_Y;
    line_point.z = 0;   
    line.points.push_back(line_point);

    line.scale.x = 0.015;
    line.color.r = 1.0;
    line.color.g = 0.0;
    line.color.b = 0.0;
    line.color.a = 1.0;

    map_pub.publish(line);

    while(ros::ok()){
        ros::spin();
    }

    return 0;
}
