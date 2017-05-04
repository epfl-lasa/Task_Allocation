#ifndef FINDER_H
#define FINDER_H

#include "PATH.h"
#include <thread>
#include "ros/ros.h"
#include <thread>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64MultiArray.h"
#include "eigen3/Eigen/Dense"
#include "multiarm_ds.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <cmath>
#include <cfloat>

//#include "Object.h"

using namespace Eigen;


bool						Position_of_the_object_recieved[N_grabbing+2];
bool 						Position_of_the_robot_recieved[2*N_robots];

ros::Subscriber 			sub_position_object_raw;
ros::Subscriber 			sub_position_object;
ros::Subscriber 			sub_command;
ros::Subscriber 			sub_G_On_object[N_grabbing];
ros::Subscriber 			sub_base_of_robot[N_robots];
ros::Subscriber 			sub_end_of_robot[N_robots];


Vector3d					P_object_raw;					// Position of the object 	(raw)V
Vector3d 					P_object_filtered;				// Position of the object 	(filter)
Vector4d					O_object_raw;					// Orientation of the object(raw)
VectorXd 					Object_State_raw;
VectorXd 					Object_Grabbing_State[N_grabbing];
Vector3d 					Intercept_point_State[N_robots];// Position of the desired intercept point
Vector3d					P_G_On_object[N_grabbing];		// Position of the grabbing positions on the object
Vector4d					O_G_On_object[N_grabbing];		// Orientation of the grabbing positions on the object

Vector3d					X_I_C;
double 						likelihood;

int 						index_of_grabbing[N_robots];

MatrixXd					predicted_object[N_grabbing];

std::thread* 				the_thread;
multiarm_ds					*Motion_G;
ros::Publisher 				pub_pos_prediction[N_grabbing];

ros::Publisher 				pub_pos_catching;
std_msgs::Float64MultiArray	msg_float;

ENUM_COMMAND 				mCommand;


double X_end[N_robots];
double Y_end[N_robots];
double Z_end[N_robots];



double X_base[N_robots];
double Y_base[N_robots];
double Z_base[N_robots];





tf::TransformBroadcaster		*tf_br;
tf::Transform 					tf_transform;
tf::Quaternion 					tf_q;



double initial_time;







void pubish_on_point_cloud(int index, MatrixXd  X)
{
	sensor_msgs::PointCloud 		pointcloud;

	pointcloud.points.resize(X.rows());
	pointcloud.header.stamp = ros::Time::now();
	pointcloud.header.frame_id="world_frame";
	for (int i=0;i<X.rows();i++)
	{
		pointcloud.points[i].x =X(i,0);
		pointcloud.points[i].y =X(i,1);
		pointcloud.points[i].z =X(i,2);
	}
	pub_pos_prediction[index].publish(pointcloud);
}
void pubish_on_tf(Vector3d  X,Vector4d  Q,std::string n)
{
	tf_transform.setOrigin( tf::Vector3(X(0),X(1), X(2)) );
	tf_q.setX(Q(0));tf_q.setY(Q(1));tf_q.setZ(Q(2));tf_q.setW(Q(3));
	tf_transform.setRotation(tf_q);
	tf_br->sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "world_frame", n));
}
bool everythingisreceived()
{
	bool flag=true;
	for (int i=0;i<2*N_robots;i++)
	{
			cout<<"Position_of_the_robot_recieved[i] "<<i<<" "<<Position_of_the_robot_recieved[i]<<endl;
		if (Position_of_the_robot_recieved[i]==false)
		{
			flag=false;
		}
	}
	for (int i=0;i<N_grabbing+2;i++)
	{
			cout<<"Position_of_the_object_recieved[i] "<<i<<" "<<Position_of_the_object_recieved[i]<<endl;
		if (Position_of_the_object_recieved[i]==false)
		{
			flag=false;
		}
	}

	return flag;
}
void reset_the_bool()
{
	cout<<"reset all the bools"<<endl;
	for (int i=0;i<2*N_robots;i++)
	{
        Position_of_the_robot_recieved[i]=false;
	}
	for (int i=0;i<N_grabbing+2;i++)
	{
		Position_of_the_object_recieved[i]=false;
	}
}

#endif
