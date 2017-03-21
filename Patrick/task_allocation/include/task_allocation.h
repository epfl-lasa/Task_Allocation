/*
 * Copyright (C) 2016 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Sina Mirrazavi
 * email:   sina.mirrazavi@epfl.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef TASK_ALLOCATION_H
#define TASK_ALLOCATION_H


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
#include "Coalition.h"


using namespace Eigen;




const double VALUATION_THRESHOLD = 1;
const int MAX_COALITION_SIZE = 3;

const int MAX_TASKS = 6;

const double OBJ_MAX_PREDICTIONTIME = 100;




class Task_allocation
{
public:
	Task_allocation();
	Task_allocation(double dt, int n_state, int max_n_bots, int max_n_tasks, MatrixXd A_V,Object_prediction_type Object_motion=Object_prediction_type::Straight);
	~Task_allocation();

	int			add_robot(Robot_agent bot);
	int 		add_task(Object Object);

	bool		set_object_state(int i, VectorXd X, VectorXd DX);
	VectorXd	get_object_state(int i);
	void 		predict_motion();
	void 		Update();

	double 		coalition_evaluate_task(int coal_size, int coalition_id, int object);
	double 		robot_evaluate_task(int i_robot, int i_object, int frame);
	void		allocate(); // patrick
	void		init_coalitions();

	friend std::ostream& operator<< (std::ostream& stream, const Task_allocation& Object);

private:

	void 	ERROR();
	void	restart_everything();

	bool	everythingisreceived();
	void	assign_the_robots();
	int 	factorial(int n);

	MatrixXd PermGenerator(int n, int k);

	Object_prediction_type Prediction_model;

	bool				catching_pos_is_found;

	int 				n_state;
	int 				max_n_robots;
	int 				n_robots;
	std::vector<Robot_agent> Robots;
	std::vector<Robot_agent*> unallocated_robots;

	int 				max_n_objects;
	int 				n_objects; // patrick

	std::vector<Object>		Objects;
	double 				dt;

	std::vector< std::vector<Coalition> > 		Coalitions;
	std::vector<Coalition*> active_coalitions;
	int 				n_frames; // patrick

};


void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);


#endif
