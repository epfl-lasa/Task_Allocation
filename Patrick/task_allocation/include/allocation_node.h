/*
 * allocation_node.h
 *
 *  Created on: Apr 7, 2017
 *      Author: patrick
 */

#ifndef IJRR_PATRICK_TASK_ALLOCATION_INCLUDE_ALLOCATION_NODE_H_
#define IJRR_PATRICK_TASK_ALLOCATION_INCLUDE_ALLOCATION_NODE_H_



#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "task_allocation.h"

void chatterCallback_end0( const geometry_msgs::Pose & msg);
void chatterCallback_end1( const geometry_msgs::Pose & msg);
void chatterCallback_end2( const geometry_msgs::Pose & msg);
void chatterCallback_end3( const geometry_msgs::Pose & msg);

void chatterCallback_base0( const geometry_msgs::Pose & msg);
void chatterCallback_base1( const geometry_msgs::Pose & msg);
void chatterCallback_base2( const geometry_msgs::Pose & msg);
void chatterCallback_base3( const geometry_msgs::Pose & msg);


void init();
void init_topics();
void prepare_task_allocator();
void add_objects_task_allocator();
void add_robots_task_allocator();


#endif /* IJRR_PATRICK_TASK_ALLOCATION_INCLUDE_ALLOCATION_NODE_H_ */
