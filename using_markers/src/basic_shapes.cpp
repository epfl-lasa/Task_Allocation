#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Pose.h"
#include "std_msgs/Int64.h"
#include "common.h"

#define UNALLOCATED_COLOR {0,1,1} // color for unallocated marker

visualization_msgs::Marker marker;
visualization_msgs::Marker marker_filtered;
visualization_msgs::Marker marker_virtual;
ros::Publisher marker_pub;
ros::Publisher marker_virtual_pub;
ros::Publisher marker_filter_pub;

double X[N_ROB] = {0, 0, 2, 2};
double Y[N_ROB] = {0, -1.25, 0, -1.25};
double Z[N_ROB] = {-0.5, -0.5, -0.5, -0.5};

int obj_done[N_OBJ];
int rob_id[N_ROB];

double colors[N_OBJ+1][3] = {{0,0,1},{1,0,0},{1,0,1},{1,1,1}, UNALLOCATED_COLOR};


visualization_msgs::Marker marker_obj[N_OBJ];
ros::Publisher marker_obj_pub[N_OBJ];

visualization_msgs::Marker marker_rob[N_ROB];
ros::Publisher marker_rob_pub[N_ROB];
double B_t_U=0.0;


void set_marker_color(visualization_msgs::Marker& mark, int index)
{
    if(index >= 0)
    {
        mark.color.r = colors[index][0];
        mark.color.g = colors[index][1];
        mark.color.b = colors[index][2];
    }
    else
    {
        mark.color.r = colors[N_OBJ][0];
        mark.color.g = colors[N_OBJ][1];
        mark.color.b = colors[N_OBJ][2];
    }
}

void set_obj_position(const geometry_msgs::Pose & msg, int index)
{

    marker_obj[index].header.stamp = ros::Time::now();

    marker_obj[index].pose.position.x = msg.position.x;
    marker_obj[index].pose.position.y = msg.position.y-B_t_U;
    marker_obj[index].pose.position.z = msg.position.z;
    marker_obj[index].pose.orientation.w = msg.orientation.w;
    marker_obj[index].pose.orientation.x = msg.orientation.x;
    marker_obj[index].pose.orientation.y = msg.orientation.y;
    marker_obj[index].pose.orientation.z = msg.orientation.z;
    marker_obj_pub[index].publish(marker_obj[index]);
}

void set_done(const std_msgs::Int64 & msg, int index)
{
    obj_done[index] = msg.data;
    if(obj_done[index])
    {
        marker_obj[index].action = visualization_msgs::Marker::DELETE;
        marker_obj_pub[index].publish(marker_obj[index]);
 //       ROS_INFO_STREAM("Basic_shapes.cpp object " << index << " is done");
    }
}

void cb_done_0(const std_msgs::Int64 & msg)
{
    set_done(msg, 0);
}

void cb_done_1(const std_msgs::Int64 & msg)
{
    set_done(msg, 1);
}

void cb_done_2(const std_msgs::Int64 & msg)
{
    set_done(msg, 2);
}

void cb_done_3(const std_msgs::Int64 & msg)
{
    set_done(msg, 3);
}


void chatterCallback_object(const geometry_msgs::Pose & msg)
{
    set_obj_position(msg, 0);
}

void chatterCallback_object_P1(const geometry_msgs::Pose & msg)
{
    set_obj_position(msg, 1);
}

void chatterCallback_object_P2(const geometry_msgs::Pose & msg)
{
    set_obj_position(msg, 2);
}

void chatterCallback_object_P3(const geometry_msgs::Pose & msg)
{
    set_obj_position(msg, 3);
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







void chatterCallback_rob0_id(const std_msgs::Int64 & msg)
{
	rob_id[0] = msg.data;
	marker_rob[0].header.stamp = ros::Time();
    set_marker_color(marker_rob[0], rob_id[0]);
    marker_rob_pub[0].publish(marker_rob[0]);

}

void chatterCallback_rob1_id(const std_msgs::Int64 & msg)
{
	rob_id[1] = msg.data;
	marker_rob[1].header.stamp = ros::Time();
    set_marker_color(marker_rob[1], rob_id[1]);
    marker_rob_pub[1].publish(marker_rob[1]);
}


void chatterCallback_rob2_id(const std_msgs::Int64 & msg)
{
	rob_id[2] = msg.data;
	marker_rob[2].header.stamp = ros::Time();
    set_marker_color(marker_rob[2], rob_id[2]);
    marker_rob_pub[2].publish(marker_rob[2]);
}


void chatterCallback_rob3_id(const std_msgs::Int64 & msg)
{
	rob_id[3] = msg.data;
	marker_rob[3].header.stamp = ros::Time();
    set_marker_color(marker_rob[3], rob_id[3]);
    marker_rob_pub[3].publish(marker_rob[3]);
}









int main( int argc, char** argv )
{
	ros::init(argc, argv, "basic_shapes");
	ros::NodeHandle n;
    int id = 0;


	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	marker_virtual_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    marker_filter_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Subscriber	sub_object = n.subscribe("/object/raw/position", 3, chatterCallback_object);
	ros::Subscriber	sub_object_virtual = n.subscribe("/object/virtual/position", 3, chatterCallback_object_virtual);
	ros::Subscriber	sub_object_fitler = n.subscribe("/object/filtered/position", 3, chatterCallback_object_filter);

    for(int i = 0; i < N_ROB; i++)
	{
		marker_rob_pub[i] = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	}

    for(int i = 0; i < N_OBJ; i++)
    {
        marker_obj_pub[i] = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
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



    for(int i = 0; i < N_OBJ; i++) // increment the ids so that we can get the IDs of the markers right
    {

        marker_obj[i].header.frame_id = "/world_frame";
        marker_obj[i].action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker_obj[i].ns = "basic_shapes";
        marker_obj[i].id = id++;
        marker_obj[i].type = shape;
        marker_obj[i].pose.orientation.x = 0.0;
        marker_obj[i].pose.orientation.y = 0.0;
        marker_obj[i].pose.orientation.z = 0.0;
        marker_obj[i].pose.orientation.w = 1.0;
        if(obj_sizes[(int)(SCENARIO)][i] == Object_sizes::SMALL)
        {
           // marker.scale = obj_scales[0];
            marker_obj[i].scale.x = 0.2f;
            marker_obj[i].scale.y = 0.2f;
            marker_obj[i].scale.z = 0.2f;
        }
        else
        {
           // marker.scale = obj_scales[1];
            marker_obj[i].scale.x = 0.4f;
            marker_obj[i].scale.y = 0.4f;
            marker_obj[i].scale.z = 0.4f;
        }

        marker_obj[i].color.r = colors[i][0];
        marker_obj[i].color.g = colors[i][1];
        marker_obj[i].color.b = colors[i][2];
        marker_obj[i].color.a = 1.0;
        marker_obj[i].pose.position.x = 0.0 ;
    }

    for(int i = 0; i < N_OBJ; i++)
        marker_obj_pub[i].publish(marker_obj[i]);



	marker_filtered.header.frame_id = "/world_frame";
	marker_filtered.action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker_filtered.ns = "basic_shapes";
    marker_filtered.id = id++;
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
    marker_virtual.id = id++;
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


    for(int i = 0; i < N_ROB; i++)
    {
        marker_rob[i].header.frame_id = "/world_frame";
        marker_rob[i].action = visualization_msgs::Marker::ADD;  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker_rob[i].ns = "basic_shapes";
        marker_rob[i].id = id++;
        marker_rob[i].type = visualization_msgs::Marker::SPHERE;
        marker_rob[i].pose.position.x = X[i];
        marker_rob[i].pose.position.y = Y[i];
        marker_rob[i].pose.position.z = Z[i];
        marker_rob[i].pose.orientation.x = 0.0;
        marker_rob[i].pose.orientation.y = 0.0;
        marker_rob[i].pose.orientation.z = 0.0;
        marker_rob[i].pose.orientation.w = 1.0;
        marker_rob[i].scale.x = 0.4;
        marker_rob[i].scale.y = 0.4;
        marker_rob[i].scale.z = 0.4;
        marker_rob[i].color.r = 1.0;
        marker_rob[i].color.g = 1.0;
        marker_rob[i].color.b = 1.0;
        marker_rob[i].color.a = 1.0;
    }

    for(int i = 0; i < N_ROB; i++)
	{
		marker_rob_pub[i].publish(marker_rob[i]);
	}

	ros::spin();
}
