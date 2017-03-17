/*
 * Coalition.h
 *
 *  Created on: 12 mars 2017
 *      Author: Patrick
 */

#ifndef COALITION_H
#define COALITION_H

#include <stdio.h>
#include <stdlib.h>
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "LPV.h"
#include "TrajectoryEstimation.h"
#include <math.h>
#include <omp.h>
#include <vector>

#include "Object.h"
#include "Robot_agent.h"




class Coalition {

public:
	Coalition();
	Coalition(int n_bots, Robot_agent** Robots, int n_tasks, Object** Tasks);
	~Coalition();
	int get_n_robots();
	int get_n_grippers();
	int add_robot(Robot_agent* bot);
	int add_task(Object* Task);
	double get_force();
	double get_coalitional_value();

	double evaluate_task(Object task);
	double compute_coalitional_value();

	friend std::ostream& operator<< (std::ostream& stream, const Coalition& Object);

private:
	const static int max_n_robots = 12;
	int n_robots;   // number of robots
	std::vector<Robot_agent*> Robots;
	//Robot_agent** Robots; // robots

	int n_grippers; // available grippers
	double force; // available force

//	Object** Tasks; // tasks
	const static int max_n_objects = 12;
	std::vector<Object*> Objects;
	int n_objects;
	double coalitional_value;



};


#endif
