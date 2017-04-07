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

	void set_id(int);
	void assign();
	int add_robot(Robot_agent* bot);
	int add_task(Object* Task);

	void print_pointer() const;
	int get_n_robots() const;
	int get_n_grippers() const;
	int get_id() const;
	std::vector<int> get_robots_id() const;
	int get_object_id() const;
	double get_force() const;
	double get_value() const;
	double get_cost() const;
	double get_weight() const;
	void compute_intercept();
	void print_intercept() const;

	double compute_value(); // updates value, cost and weight

	friend std::ostream& operator<< (std::ostream& stream, const Coalition& Object);

private:

	bool is_feasible(Object& obj);

	int id;

	const static int max_n_robots = 12;
	int n_robots;   // number of robots
	std::vector<Robot_agent*> Robots;

	int n_grippers; // available grippers
	double force; // available force

	const static int max_n_objects = 12;
	std::vector<Object*> Objects;

	Object* des_obj; // desired object for this coalition
	int n_objects;
	double value;
	double cost;
	double weight;

};


#endif
