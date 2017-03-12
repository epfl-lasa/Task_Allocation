/*
 * Robot.h
 *
 *  Created on: 12 mars 2017
 *      Author: Patrick
 */

#ifndef ROBOT_H_
#define ROBOT_H_


#include <stdio.h>
#include <stdlib.h>
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "LPV.h"
#include "TrajectoryEstimation.h"
#include <math.h>
#include  <omp.h>


class Robot {

public:
	bool get_workspace_set();
	bool get_state_set();
	bool get_LPV_set();
	bool get_desired_state_set();
	void get_coordination_allocation(int index, double& x);
	void get_coordination_parameter(double& x);
	void get_state();

	void Set_initial_state(int index, Vector3d X);
	void Set_state(int index, VectorXd X);
	void Set_first_primitive_desired_position(int index, VectorXd X, VectorXd DX);

private:
	bool Workspace_model_is_set;
	bool state_is_set;
	bool LPV_is_set;
	bool desired_state_is_set;
	int 		index_of_grabbing_posititon;
	GMM 		Workspace_model;
	VectorXd 	X_Base; 			//Position of the base of the robot with respect to the world frame
	Vector3d 	X_Initial_pose; 	// Initial position of the end-effector of the robot with respect to the world frame
	LPV			Dynamic;
	VectorXd 	ATX;		//To simplify the calculations!
	VectorXd 	X; 					//State of the robot in the world frame
	VectorXd 	X_INTERCEPT; 			//State of the robot in the world frame
	VectorXd 	DX;
	VectorXd 	X_I_C;		// Desired intercept point of the i th robot
	VectorXd 	X_F_P;		// First primitive desired state
	VectorXd 	DX_F_P;	// First primitive desired D-state
	VectorXd 	X_d;		//The desired state of the robot
	VectorXd 	DX_d;		//The derivative of the desired state of the robot
	double 		tau;		//Coordination allocation
	double		Dtau;		//Derivative of Coordination allocation
	double		DDtau;		//Derivative of Coordination allocation
	MatrixXd	Probability_of_catching;
	double 		M[N_coordination_allocation_paramerets];


	int 		n_grippers; // number of grippers on this robot, typically 1
	double 		force; // force of the robot
	bool		is_assigned; // is the robot assigned to a task?
};


#endif
