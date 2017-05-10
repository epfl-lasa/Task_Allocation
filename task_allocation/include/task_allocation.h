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



/* TODO
 *
 *
 * Make sure the indices of the robots between this object and the multiarm_ds are the same
 * Modify the algorithm to only allow one coalition of more than 1 robot
 * Send the coordination parameters to the multiarm_ds
 * Send the intercept point for the robots with a single item
 */


#ifndef TASK_ALLOCATION_H
#define TASK_ALLOCATION_H


#include <stdio.h>
#include <stdlib.h>
#include "eigen3/Eigen/Dense"
#include "ros/ros.h"
#include <math.h>
#include <vector>
#include "Coalition.h"
#include "common.h"


using namespace Eigen;


class Task_allocation
{
public:
	Task_allocation();
	Task_allocation(double max_time_, double dt, int n_state, Object_prediction_type Object_motion=Object_prediction_type::Straight);
//	~Task_allocation();

	int			add_robot(Robot_agent* bot);
	int 		add_task(Object* Object);

	bool		set_object_state(int i, VectorXd X, VectorXd DX);
	bool		set_robot_state(int i, VectorXd X_);
	VectorXd	get_object_state(int i);
	void 		predict_motion();
	void		update_rob_business();
	void		build_coalitions(); //this makes "Coalitions" to hold all coalitions with the currently unallocated robots
	void		clear_coalitions(); //this resets the coalitions, resets the unallocated robots and active coalitions
	void		compute_intercepts();
	void		allocate();
	void		multi_frame_allocation();



	double 		get_dt() const;
	double 		get_max_time() const;
	int 		get_n_coals() const;
	int 		get_robot_target(int i) const;



	void		print_obj() const;
	void		print_coalitions() const;
	void 		print_bases() const;
	void		print_intercepts() const;
	void 		update_objects_value();

	void		compute_coordination();
	MatrixXd	get_targets();
	std::vector<double> get_coordinations();
    std::vector<Coalition> get_coalitions() const;

	friend std::ostream& operator<< (std::ostream& stream, const Task_allocation& Object);
    bool        check_dupe(const VectorXd& rowA, const VectorXd& rowB) const;
private:

	void 	ERROR();
	void	restart_everything();
	void	evaluate_coalitions();

	void	assign_the_robots();
	int 	factorial(int n);

	MatrixXd targets;
	std::vector<double> coordinations;


	MatrixXd PermGenerator(int n, int k);

	Object_prediction_type Prediction_model;


	int 				n_state;

	int 				n_robots;
	std::vector<Robot_agent*> Robots;
	std::vector<Robot_agent*> unallocated_robots;

	int 				n_objects;
	std::vector<Object*>		Objects;

	double				max_pred_time;
	double 				dt;
	int 				n_frames;

	std::vector< std::vector<Coalition> > 		Coalitions;
	std::vector<Coalition> active_coalitions;
	int 			n_coalitions;
//	multiarm_ds* 	Multi_ds;
};


void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);


#endif
