/*
 * allocation_node.h
 *
 *  Created on: Apr 7, 2017
 *      Author: patrick
 */

#ifndef IJRR_PATRICK_TASK_ALLOCATION_INCLUDE_ALLOCATION_NODE_H_
#define IJRR_PATRICK_TASK_ALLOCATION_INCLUDE_ALLOCATION_NODE_H_

#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "task_allocation.h"
#include <string>

// This is hideous, should just include "PATH.h"
std::string Common_path  = "/home/patrick/lasa-project/catkin_ws/src/IJRR/simple_example_two_kuka/data"; // dirty

string addTwochar(string folder_path, string b, int integer1=-1,int integer2=-1)
{
	std::string str;
	str.append(folder_path);
	str.append(b);
	if (integer1!=-1)
	{
		ostringstream convert;
		convert << integer1;
		str.append(convert.str());
	}
	if (integer2!=-1)
	{
		ostringstream convert;
		convert << integer2;
		str.append(convert.str());
	}
	str.append(".txt");
	return str;
}



void init();
void init_topics();
void prepare_task_allocator();
void add_objects_task_allocator();
void add_robots_task_allocator();




/*

void chatterCallback_obj_pos0( const geometry_msgs::Pose & msg);
void chatterCallback_obj_pos1( const geometry_msgs::Pose & msg);
void chatterCallback_obj_pos2( const geometry_msgs::Pose & msg);
void chatterCallback_obj_pos3( const geometry_msgs::Pose & msg);


void chatterCallback_end0( const geometry_msgs::Pose & msg);
void chatterCallback_end1( const geometry_msgs::Pose & msg);
void chatterCallback_end2( const geometry_msgs::Pose & msg);
void chatterCallback_end3( const geometry_msgs::Pose & msg);

void chatterCallback_base0( const geometry_msgs::Pose & msg);
void chatterCallback_base1( const geometry_msgs::Pose & msg);
void chatterCallback_base2( const geometry_msgs::Pose & msg);
void chatterCallback_base3( const geometry_msgs::Pose & msg);
*/


#endif /* IJRR_PATRICK_TASK_ALLOCATION_INCLUDE_ALLOCATION_NODE_H_ */
