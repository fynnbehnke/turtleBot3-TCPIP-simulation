#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>

#define BORDER_X -0.58
#define BORDER_Y 0.195
#define BORDER_LENGTH 1.189
#define BORDER_WIDTH 0.841

#define CYLINDER_X 0.0
#define CYLINDER_Y 0.0
#define CYLINDER_HEIGHT 1.0
#define CYLINDER_RADIUS 0.0315

#define LINE_X -0.499
#define LINE_Y -0.241
#define LINE_LENGTH 1.043

#define CIRCLE_X -0.343
#define CIRCLE_Y -0.409
#define CIRCLE_RADIUS 0.15

#define SQUARE_X -0.106
#define SQUARE_Y -0.26
#define SQUARE_LENGTH 0.3

#define TRIANGLE_X 0.409
#define TRIANGLE_Y -0.26
#define TRIANGLE_LENGTH_SIDE 0.3
#define TRIANGLE_LENGTH_BOT 0.3


int main(int argc, char** argv){
    
    ros::init(argc, argv, "marker_pub_node");

    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<visualization_msgs::MarkerArray>("/apr/map/marker", 1, true);

    visualization_msgs::MarkerArray map_marker;

    // This marks the border of the map
    geometry_msgs::Point border_point;
    visualization_msgs::Marker border;

    border.header.frame_id = "map";
    border.id = 5;
    border.type = visualization_msgs::Marker::LINE_STRIP;
    
    border_point.x = BORDER_X;
    border_point.y = BORDER_Y;
    border_point.z = 0;   
    border.points.push_back(border_point);

    border_point.x =  BORDER_X;
    border_point.y =  BORDER_Y-BORDER_WIDTH;
    border_point.z = 0;   
    border.points.push_back(border_point);

    border_point.x = BORDER_X+BORDER_LENGTH;
    border_point.y = BORDER_Y-BORDER_WIDTH;
    border_point.z = 0;   
    border.points.push_back(border_point);

    border_point.x = BORDER_X+BORDER_LENGTH;
    border_point.y = BORDER_Y;
    border_point.z = 0;   
    border.points.push_back(border_point);

    border_point.x = BORDER_X;
    border_point.y = BORDER_Y;
    border_point.z = 0;   
    border.points.push_back(border_point);
    
    border.pose.orientation.w = 1;
    border.pose.orientation.x = 0;
    border.pose.orientation.y = 0;
    border.pose.orientation.z = 0;
    border.scale.x = 0.015;
    border.color.r = 0.0;
    border.color.g = 0.0;
    border.color.b = 0.0;
    border.color.a = 1.0;


    // This marks the Roll used for Localization
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


    // This marks the Line to follow
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

    line.pose.orientation.w = 1;
    line.pose.orientation.x = 0;
    line.pose.orientation.y = 0;
    line.pose.orientation.z = 0;
    line.scale.x = 0.015;
    line.color.r = 1.0;
    line.color.g = 0.0;
    line.color.b = 0.0;
    line.color.a = 1.0;


    // This marks the circle to follow
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

    circle.pose.orientation.w = 1;
    circle.pose.orientation.x = 0;
    circle.pose.orientation.y = 0;
    circle.pose.orientation.z = 0;
    circle.scale.x = 0.015;
    circle.color.r = 1.0;
    circle.color.g = 0.0;
    circle.color.b = 0.0;
    circle.color.a = 1.0;


    // This marks the square to follow
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
    
    square.pose.orientation.w = 1;
    square.pose.orientation.x = 0;
    square.pose.orientation.y = 0;
    square.pose.orientation.z = 0;
    square.scale.x = 0.015;
    square.color.r = 1.0;
    square.color.g = 0.0;
    square.color.b = 0.0;
    square.color.a = 1.0;


    // This marks the triangle to follow
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
    
    triangle.pose.orientation.w = 1;
    triangle.pose.orientation.x = 0;
    triangle.pose.orientation.y = 0;
    triangle.pose.orientation.z = 0;
    triangle.scale.x = 0.015;
    triangle.color.r = 1.0;
    triangle.color.g = 0.0;
    triangle.color.b = 0.0;
    triangle.color.a = 1.0;


    // Add all markers and publish them
    map_marker.markers.push_back(border);
    map_marker.markers.push_back(cylinder);
    map_marker.markers.push_back(line);
    map_marker.markers.push_back(circle);
    map_marker.markers.push_back(square);
    map_marker.markers.push_back(triangle);

    map_pub.publish(map_marker);

    while(ros::ok()){
        ros::spin();
    }

    return 0;
}
