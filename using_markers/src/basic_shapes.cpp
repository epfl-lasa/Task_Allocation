


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Pose.h"
#include "std_msgs/Int64.h"

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




const int n_bots = 4;
const int n_obj = 4;
int obj_done[n_obj];
int rob_id[n_bots];

enum class Object_size {SMALL, LARGE};
//const geometry_msgs::Vector3 obj_scales[2] = {geometry_msgs::Vector3(0.2,0.2,0.2), geometry_msgs::Vector3(0.4,0.4,0.4)};
const Object_size obj_sizes[n_obj] = {Object_size::LARGE, Object_size::SMALL, Object_size::SMALL, Object_size::SMALL};

visualization_msgs::Marker marker_rob[n_bots];
ros::Publisher marker_rob_pub[n_bots];
double B_t_U=0.0;


void cb_done_0(const std_msgs::Int64 & msg)
{
	obj_done[0] = msg.data;
	if(obj_done[0])
	{
		marker.action = visualization_msgs::Marker::DELETE;
		marker_pub.publish(marker);
	}
}

void cb_done_1(const std_msgs::Int64 & msg)
{
	obj_done[1] = msg.data;
	if(obj_done[1])
	{
		marker_p1.action = visualization_msgs::Marker::DELETE;
		marker_p1_pub.publish(marker_p1);
	}
}

void cb_done_2(const std_msgs::Int64 & msg)
{
	obj_done[2] = msg.data;
	if(obj_done[2])
	{
		marker_p2.action = visualization_msgs::Marker::DELETE;
		marker_p2_pub.publish(marker_p2);
	}
}

void cb_done_3(const std_msgs::Int64 & msg)
{
	obj_done[3] = msg.data;
	if(obj_done[3])
	{
		marker_p3.action = visualization_msgs::Marker::DELETE;
		marker_p3_pub.publish(marker_p3);
	}
}


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



void chatterCallback_rob0_id(const std_msgs::Int64 & msg)
{
	rob_id[0] = msg.data;
	marker_rob[0].header.stamp = ros::Time();
	switch(rob_id[0])
	{
		case 0:
			marker_rob[0].color.r = 0.0f;
			marker_rob[0].color.g = 0.0f;
			marker_rob[0].color.b = 1.0f;
			break;
		case 1:
			marker_rob[0].color.r = 1.0f;
			marker_rob[0].color.g = 0.0f;
			marker_rob[0].color.b = 0.0f;
			break;
		case 2:
			marker_rob[0].color.r = 1.0f;
			marker_rob[0].color.g = 0.0f;
			marker_rob[0].color.b = 1.0f;
			break;
		case 3:
			marker_rob[0].color.r = 1.0f;
			marker_rob[0].color.g = 1.0f;
			marker_rob[0].color.b = 1.0f;
			break;
		default:
			marker_rob[0].color.r = 0.0f;
			marker_rob[0].color.g = 1.0f;
			marker_rob[0].color.b = 1.0f;
			break;
	}
	marker_rob_pub[0].publish(marker_rob[0]);

}

void chatterCallback_rob1_id(const std_msgs::Int64 & msg)
{
	rob_id[1] = msg.data;
	marker_rob[1].header.stamp = ros::Time();
	switch(rob_id[1])
	{
		case 0:
			marker_rob[1].color.r = 0.0f;
			marker_rob[1].color.g = 0.0f;
			marker_rob[1].color.b = 1.0f;
			break;
		case 1:
			marker_rob[1].color.r = 1.0f;
			marker_rob[1].color.g = 0.0f;
			marker_rob[1].color.b = 0.0f;
			break;
		case 2:
			marker_rob[1].color.r = 1.0f;
			marker_rob[1].color.g = 0.0f;
			marker_rob[1].color.b = 1.0f;
			break;
		case 3:
			marker_rob[1].color.r = 1.0f;
			marker_rob[1].color.g = 1.0f;
			marker_rob[1].color.b = 1.0f;
			break;
		default:
			marker_rob[1].color.r = 0.0f;
			marker_rob[1].color.g = 1.0f;
			marker_rob[1].color.b = 1.0f;
			break;
	}
	marker_rob_pub[1].publish(marker_rob[1]);
//	std::cout << "got a robot id marker" << std::endl;
}


void chatterCallback_rob2_id(const std_msgs::Int64 & msg)
{
	rob_id[2] = msg.data;
	marker_rob[2].header.stamp = ros::Time();
	switch(rob_id[2])
	{
		case 0:
			marker_rob[2].color.r = 0.0f;
			marker_rob[2].color.g = 0.0f;
			marker_rob[2].color.b = 1.0f;
			break;
		case 1:
			marker_rob[2].color.r = 1.0f;
			marker_rob[2].color.g = 0.0f;
			marker_rob[2].color.b = 0.0f;
			break;
		case 2:
			marker_rob[2].color.r = 1.0f;
			marker_rob[2].color.g = 0.0f;
			marker_rob[2].color.b = 1.0f;
			break;
		case 3:
			marker_rob[2].color.r = 1.0f;
			marker_rob[2].color.g = 1.0f;
			marker_rob[2].color.b = 1.0f;
			break;
		default:
			marker_rob[2].color.r = 0.0f;
			marker_rob[2].color.g = 1.0f;
			marker_rob[2].color.b = 1.0f;
			break;
	}
	marker_rob_pub[2].publish(marker_rob[2]);
//	std::cout << "got a robot id marker" << std::endl;
}


void chatterCallback_rob3_id(const std_msgs::Int64 & msg)
{
	rob_id[3] = msg.data;

	marker_rob[3].header.stamp = ros::Time();
	switch(rob_id[3])
	{
		case 0:
			marker_rob[3].color.r = 0.0f;
			marker_rob[3].color.g = 0.0f;
			marker_rob[3].color.b = 1.0f;
			break;
		case 1:
			marker_rob[3].color.r = 1.0f;
			marker_rob[3].color.g = 0.0f;
			marker_rob[3].color.b = 0.0f;
			break;
		case 2:
			marker_rob[3].color.r = 1.0f;
			marker_rob[3].color.g = 0.0f;
			marker_rob[3].color.b = 1.0f;
			break;
		case 3:
			marker_rob[3].color.r = 1.0f;
			marker_rob[3].color.g = 1.0f;
			marker_rob[3].color.b = 1.0f;
			break;
		default:
			marker_rob[3].color.r = 0.0f;
			marker_rob[3].color.g = 1.0f;
			marker_rob[3].color.b = 1.0f;
			break;
	}
	marker_rob_pub[3].publish(marker_rob[3]);
//	std::cout << "got a robot id marker" << std::endl;
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

	for(int i = 0; i < n_bots; i++)
	{
		marker_rob_pub[i] = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	}

	ros::Subscriber sub_rob0_id = n.subscribe("/robotsPat/targetID/0", 1, chatterCallback_rob0_id);
	ros::Subscriber sub_rob1_id = n.subscribe("/robotsPat/targetID/1", 1, chatterCallback_rob1_id);
	ros::Subscriber sub_rob2_id = n.subscribe("/robotsPat/targetID/2", 1, chatterCallback_rob2_id);
	ros::Subscriber sub_rob3_id = n.subscribe("/robotsPat/targetID/3", 1, chatterCallback_rob3_id);

	ros::Subscriber	sub_object_p1 = n.subscribe("/object/p1/position", 1, chatterCallback_object_P1);
	ros::Subscriber	sub_object_p2 = n.subscribe("/object/p2/position", 1, chatterCallback_object_P2);
	ros::Subscriber	sub_object_p3 = n.subscribe("/object/p3/position", 1, chatterCallback_object_P3);

	ros::Subscriber sub_obj_done_0 = n.subscribe("/object/p0/done", 1, cb_done_0);
	ros::Subscriber sub_obj_done_1 = n.subscribe("/object/p1/done", 1, cb_done_1);
	ros::Subscriber sub_obj_done_2 = n.subscribe("/object/p2/done", 1, cb_done_2);
	ros::Subscriber sub_obj_done_3 = n.subscribe("/object/p3/done", 1, cb_done_3);


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
    if(obj_sizes[0] == Object_size::SMALL)
    {
       // marker.scale = obj_scales[0];
        marker.scale.x = 0.2f;
        marker.scale.y = 0.2f;
        marker.scale.z = 0.2f;
    }
    else
    {
       // marker.scale = obj_scales[1];
        marker.scale.x = 0.4f;
        marker.scale.y = 0.4f;
        marker.scale.z = 0.4f;
    }

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
    if(obj_sizes[1] == Object_size::SMALL)
    {
       // marker.scale = obj_scales[0];
        marker_p1.scale.x = 0.2f;
        marker_p1.scale.y = 0.2f;
        marker_p1.scale.z = 0.2f;
    }
    else
    {
    //    marker.scale = obj_scales[1];
        marker_p1.scale.x = 0.4f;
        marker_p1.scale.y = 0.4f;
        marker_p1.scale.z = 0.4f;
    }
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
    if(obj_sizes[2] == Object_size::SMALL)
    {
       // marker.scale = obj_scales[0];
        marker_p2.scale.x = 0.2f;
        marker_p2.scale.y = 0.2f;
        marker_p2.scale.z = 0.2f;
    }
    else
    {
    //    marker.scale = obj_scales[1];
        marker_p2.scale.x = 0.4f;
        marker_p2.scale.y = 0.4f;
        marker_p2.scale.z = 0.4f;
    }
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
    if(obj_sizes[3] == Object_size::SMALL)
    {
       // marker.scale = obj_scales[0];
        marker_p3.scale.x = 0.2f;
        marker_p3.scale.y = 0.2f;
        marker_p3.scale.z = 0.2f;
    }
    else
    {
    //    marker.scale = obj_scales[1];
        marker_p3.scale.x = 0.4f;
        marker_p3.scale.y = 0.4f;
        marker_p3.scale.z = 0.4f;
    }
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





	marker_rob[0].header.frame_id = "/world_frame";
	marker_rob[0].action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker_rob[0].ns = "basic_shapes";
	marker_rob[0].id = 6;
	marker_rob[0].type = visualization_msgs::Marker::SPHERE;
	marker_rob[0].pose.position.x = 0;
	marker_rob[0].pose.position.y = -0;
	marker_rob[0].pose.position.z = -0.25;
	marker_rob[0].pose.orientation.x = 0.0;
	marker_rob[0].pose.orientation.y = 0.0;
	marker_rob[0].pose.orientation.z = 0.0;
	marker_rob[0].pose.orientation.w = 1.0;
	marker_rob[0].scale.x = 0.4;
	marker_rob[0].scale.y = 0.4;
	marker_rob[0].scale.z = 0.4;
	marker_rob[0].color.r = 1.0;
	marker_rob[0].color.g = 1.0;
	marker_rob[0].color.b = 1.0;
	marker_rob[0].color.a = 1.0;


	marker_rob[1].header.frame_id = "/world_frame";
	marker_rob[1].action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker_rob[1].ns = "basic_shapes";
	marker_rob[1].id = 7;
	marker_rob[1].type = visualization_msgs::Marker::SPHERE;
	marker_rob[1].pose.position.x = 0;
	marker_rob[1].pose.position.y = -1.25;
	marker_rob[1].pose.position.z = -0.25;
	marker_rob[1].pose.orientation.x = 0.0;
	marker_rob[1].pose.orientation.y = 0.0;
	marker_rob[1].pose.orientation.z = 0.0;
	marker_rob[1].pose.orientation.w = 1.0;
	marker_rob[1].scale.x = 0.4;
	marker_rob[1].scale.y = 0.4;
	marker_rob[1].scale.z = 0.4;
	marker_rob[1].color.r = 1.0;
	marker_rob[1].color.g = 1.0;
	marker_rob[1].color.b = 1.0;
	marker_rob[1].color.a = 1.0;


	marker_rob[2].header.frame_id = "/world_frame";
	marker_rob[2].action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker_rob[2].ns = "basic_shapes";
	marker_rob[2].id = 8;
	marker_rob[2].type = visualization_msgs::Marker::SPHERE;
	marker_rob[2].pose.position.x = 2;
	marker_rob[2].pose.position.y = -0;
	marker_rob[2].pose.position.z = -0.25;
	marker_rob[2].pose.orientation.x = 0.0;
	marker_rob[2].pose.orientation.y = 0.0;
	marker_rob[2].pose.orientation.z = 0.0;
	marker_rob[2].pose.orientation.w = 1.0;
	marker_rob[2].scale.x = 0.4;
	marker_rob[2].scale.y = 0.4;
	marker_rob[2].scale.z = 0.4;
	marker_rob[2].color.r = 1.0;
	marker_rob[2].color.g = 1.0;
	marker_rob[2].color.b = 1.0;
	marker_rob[2].color.a = 1.0;



	marker_rob[3].header.frame_id = "/world_frame";
	marker_rob[3].action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker_rob[3].ns = "basic_shapes";
	marker_rob[3].id = 9;
	marker_rob[3].type = visualization_msgs::Marker::SPHERE;
	marker_rob[3].pose.position.x = 2;
	marker_rob[3].pose.position.y = -1.25;
	marker_rob[3].pose.position.z = -0.25;
	marker_rob[3].pose.orientation.x = 0.0;
	marker_rob[3].pose.orientation.y = 0.0;
	marker_rob[3].pose.orientation.z = 0.0;
	marker_rob[3].pose.orientation.w = 1.0;
	marker_rob[3].scale.x = 0.4;
	marker_rob[3].scale.y = 0.4;
	marker_rob[3].scale.z = 0.4;
	marker_rob[3].color.r = 1.0;
	marker_rob[3].color.g = 1.0;
	marker_rob[3].color.b = 1.0;
	marker_rob[3].color.a = 1.0;



	for(int i = 0; i < n_bots; i++)
	{
		marker_rob_pub[i].publish(marker_rob[i]);
	}

	ros::spin();
}
