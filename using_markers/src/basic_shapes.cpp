


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Pose.h"


visualization_msgs::Marker marker_p1;
visualization_msgs::Marker marker_p2;
visualization_msgs::Marker marker_p3;
ros::Publisher marker_p1_pub;
ros::Publisher marker_p2_pub;
ros::Publisher marker_p3_pub;

visualization_msgs::Marker marker;
visualization_msgs::Marker marker_filtered;
visualization_msgs::Marker marker_virtual;
ros::Publisher marker_pub;
ros::Publisher marker_virtual_pub;
ros::Publisher marker_filter_pub;

double B_t_U=0.0;

void chatterCallback_object(const geometry_msgs::Pose & msg)
{

	marker.header.stamp = ros::Time::now();

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

	marker.pose.position.x = msg.position.x;
	marker.pose.position.y = msg.position.y-B_t_U;
	marker.pose.position.z = msg.position.z;
	marker.pose.orientation.w = msg.orientation.w;
	marker.pose.orientation.x = msg.orientation.x;
	marker.pose.orientation.y = msg.orientation.y;
	marker.pose.orientation.z = msg.orientation.z;
	marker_pub.publish(marker);
}

void chatterCallback_object_virtual(const geometry_msgs::Pose & msg)
{

	marker_virtual.header.stamp = ros::Time::now();

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

	marker_virtual.pose.position.x = msg.position.x;
	marker_virtual.pose.position.y = msg.position.y-B_t_U;
	marker_virtual.pose.position.z = msg.position.z;
	marker_virtual.pose.orientation.w = msg.orientation.w;
	marker_virtual.pose.orientation.x = msg.orientation.x;
	marker_virtual.pose.orientation.y = msg.orientation.y;
	marker_virtual.pose.orientation.z = msg.orientation.z;
	marker_virtual_pub.publish(marker_virtual);

}


void chatterCallback_object_filter(const geometry_msgs::Pose & msg)
{

	marker_filtered.header.stamp = ros::Time::now();

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

	marker_filtered.pose.position.x = msg.position.x;
	marker_filtered.pose.position.y = msg.position.y-B_t_U;
	marker_filtered.pose.position.z = msg.position.z;
	marker_filtered.pose.orientation.w = msg.orientation.w;
	marker_filtered.pose.orientation.x = msg.orientation.x;
	marker_filtered.pose.orientation.y = msg.orientation.y;
	marker_filtered.pose.orientation.z = msg.orientation.z;
	//marker_filter_pub.publish(marker_filtered);
}


void chatterCallback_object_P1(const geometry_msgs::Pose & msg)
{

	marker_p1.header.stamp = ros::Time::now();

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

	marker_p1.pose.position.x = msg.position.x;
	marker_p1.pose.position.y = msg.position.y-B_t_U;
	marker_p1.pose.position.z = msg.position.z;
	marker_p1.pose.orientation.w = msg.orientation.w;
	marker_p1.pose.orientation.x = msg.orientation.x;
	marker_p1.pose.orientation.y = msg.orientation.y;
	marker_p1.pose.orientation.z = msg.orientation.z;
	marker_p1_pub.publish(marker_p1);
}


void chatterCallback_object_P2(const geometry_msgs::Pose & msg)
{

	marker_p2.header.stamp = ros::Time::now();

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

	marker_p2.pose.position.x = msg.position.x;
	marker_p2.pose.position.y = msg.position.y-B_t_U;
	marker_p2.pose.position.z = msg.position.z;
	marker_p2.pose.orientation.w = msg.orientation.w;
	marker_p2.pose.orientation.x = msg.orientation.x;
	marker_p2.pose.orientation.y = msg.orientation.y;
	marker_p2.pose.orientation.z = msg.orientation.z;
	marker_p2_pub.publish(marker_p2);
}

void chatterCallback_object_P3(const geometry_msgs::Pose & msg)
{

	marker_p3.header.stamp = ros::Time::now();

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

	marker_p3.pose.position.x = msg.position.x;
	marker_p3.pose.position.y = msg.position.y-B_t_U;
	marker_p3.pose.position.z = msg.position.z;
	marker_p3.pose.orientation.w = msg.orientation.w;
	marker_p3.pose.orientation.x = msg.orientation.x;
	marker_p3.pose.orientation.y = msg.orientation.y;
	marker_p3.pose.orientation.z = msg.orientation.z;
	marker_p3_pub.publish(marker_p3);
}








int main( int argc, char** argv )
{
	ros::init(argc, argv, "basic_shapes");
	ros::NodeHandle n;
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	marker_virtual_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	marker_filter_pub= n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Subscriber	sub_object = n.subscribe("/object/raw/position", 3, chatterCallback_object);
	ros::Subscriber	sub_object_virtual = n.subscribe("/object/virtual/position", 3, chatterCallback_object_virtual);
	ros::Subscriber	sub_object_fitler = n.subscribe("/object/filtered/position", 3, chatterCallback_object_filter);

	marker_p1_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	marker_p2_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	marker_p3_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	ros::Subscriber	sub_object_p1 = n.subscribe("/object/p1/position", 3, chatterCallback_object_P1);
	ros::Subscriber	sub_object_p2 = n.subscribe("/object/p2/position", 3, chatterCallback_object_P2);
	ros::Subscriber	sub_object_p3 = n.subscribe("/object/p3/position", 3, chatterCallback_object_P3);


	// Set our initial shape type to be a cube
	uint32_t   shape = visualization_msgs::Marker::CUBE;

	marker.header.frame_id = "/world_frame";
	marker.action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = shape;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	double scale=0.5;
	marker.scale.x = 0.4;
	marker.scale.y = 0.4;
	marker.scale.z = 0.4;
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;
	marker.pose.position.x = 0.0 ;


	marker_p1.header.frame_id = "/world_frame";
	marker_p1.action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker_p1.ns = "basic_shapes";
	marker_p1.id = 1;
	marker_p1.type = shape;
	marker_p1.pose.orientation.x = 0.0;
	marker_p1.pose.orientation.y = 0.0;
	marker_p1.pose.orientation.z = 0.0;
	marker_p1.pose.orientation.w = 1.0;
	marker_p1.scale.x = 0.2;
	marker_p1.scale.y = 0.2;
	marker_p1.scale.z = 0.2;
	marker_p1.color.r = 1.0f;
	marker_p1.color.g = 0.0f;
	marker_p1.color.b = 0.0f;
	marker_p1.color.a = 1.0;
	marker_p1.pose.position.x = 0.0 ;

	marker_p2.header.frame_id = "/world_frame";
	marker_p2.action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker_p2.ns = "basic_shapes";
	marker_p2.id = 2;
	marker_p2.type = shape;
	marker_p2.pose.orientation.x = 0.0;
	marker_p2.pose.orientation.y = 0.0;
	marker_p2.pose.orientation.z = 0.0;
	marker_p2.pose.orientation.w = 1.0;
	marker_p2.scale.x = 0.2;
	marker_p2.scale.y = 0.2;
	marker_p2.scale.z = 0.2;
	marker_p2.color.r = 1.0f;
	marker_p2.color.g = 0.0f;
	marker_p2.color.b = 1.0f;
	marker_p2.color.a = 1.0;
	marker_p2.pose.position.x = 0.0 ;


	marker_p3.header.frame_id = "/world_frame";
	marker_p3.action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker_p3.ns = "basic_shapes";
	marker_p3.id = 3;
	marker_p3.type = shape;
	marker_p3.pose.orientation.x = 0.0;
	marker_p3.pose.orientation.y = 0.0;
	marker_p3.pose.orientation.z = 0.0;
	marker_p3.pose.orientation.w = 1.0;
	marker_p3.scale.x = 0.2;
	marker_p3.scale.y = 0.2;
	marker_p3.scale.z = 0.2;
	marker_p3.color.r = 1.0f;
	marker_p3.color.g = 1.0f;
	marker_p3.color.b = 1.0f;
	marker_p3.color.a = 1.0;
	marker_p3.pose.position.x = 0.0 ;







	marker_filtered.header.frame_id = "/world_frame";
	marker_filtered.action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker_filtered.ns = "basic_shapes";
	marker_filtered.id = 4;
	marker_filtered.type = shape;
	marker_filtered.pose.orientation.x = 0.0;
	marker_filtered.pose.orientation.y = 0.0;
	marker_filtered.pose.orientation.z = 0.0;
	marker_filtered.pose.orientation.w = 1.0;
	scale=0.5;
	marker_filtered.scale.x = 0.4;
	marker_filtered.scale.y = 0.4;
	marker_filtered.scale.z = 0.4;
	marker_filtered.color.r = 1.0f;
	marker_filtered.color.g = 0.0f;
	marker_filtered.color.b = 0.0f;
	marker_filtered.color.a = 1.0;
	marker_filtered.pose.position.x = 0.0 ;




	marker_virtual.header.frame_id = "/world_frame";
	marker_virtual.action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker_virtual.ns = "basic_shapes";
	marker_virtual.id = 5;
	marker_virtual.type = shape;
	marker_virtual.pose.orientation.x = 0.0;
	marker_virtual.pose.orientation.y = 0.0;
	marker_virtual.pose.orientation.z = 0.0;
	marker_virtual.pose.orientation.w = 1.0;
	marker_virtual.scale.x = 0.4;
	marker_virtual.scale.y = 0.4;
	marker_virtual.scale.z = 0.1;
	marker_virtual.color.r = 0.0f;
	marker_virtual.color.g = 1.0f;
	marker_virtual.color.b = 0.0f;
	marker_virtual.color.a = 1.0;
	marker_virtual.pose.position.x = 0.0 ;
	marker_pub.publish(marker);
	marker_virtual_pub.publish(marker_virtual);


	marker_p1_pub.publish(marker_p1);
	marker_p2_pub.publish(marker_p2);
	marker_p3_pub.publish(marker_p3);
	ros::spin();
}
