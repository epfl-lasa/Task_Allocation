/*
 * Robot_agent.h
 *
 *  Created on: 12 mars 2017
 *      Author: Patrick
 */

#ifndef ROBOT_AGENT_H_
#define ROBOT_AGENT_H_


#include <stdio.h>
#include <stdlib.h>
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "LPV.h"
#include "TrajectoryEstimation.h"
#include <math.h>
#include  <omp.h>

using namespace Eigen;

class Robot_agent {

public:
	Robot_agent();
	Robot_agent(GMM model, VectorXd base, Vector3d initial, LPV dyn_mod, VectorXd ATX_,
			VectorXd X_, VectorXd X_intercept_, VectorXd DX_, VectorXd X_I_C_, VectorXd X_F_P_,
			VectorXd DX_F_P_, VectorXd X_d_, VectorXd DX_d_, double tau_, double Dtau_, double DDtau_, MatrixXd Prob_of_catching_, int n_grippers_, double force);
	~Robot_agent();
	bool get_workspace_set();
	bool get_state_set();
	bool get_LPV_set();
	bool get_desired_state_set();
	void get_coordination_allocation(int index, double& x);
	void get_coordination_parameter(double& x);
	void get_state(VectorXd& X);
	int get_n_grippers();
	double get_force();

	void set_initial_state(Vector3d X);
	void set_state(VectorXd X);
	void set_first_primitive_desired_position(VectorXd X, VectorXd DX);

private:
	bool workspace_model_is_set;
	bool state_is_set;
	bool LPV_is_set;
	bool desired_state_is_set;
	int 		index_of_grabbing_position;
	GMM 		Workspace_model;
	VectorXd 	X_Base; 			//Position of the base of the robot with respect to the world frame
	Vector3d 	X_initial_pose; 	// Initial position of the end-effector of the robot with respect to the world frame
	LPV			Dynamic;
	VectorXd 	ATX;		//To simplify the calculations!
	VectorXd 	X; 					//State of the robot in the world frame
	VectorXd 	X_intercept; 			//State of the robot in the world frame
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
	double*		M;


	int 		n_grippers; // number of grippers on this robot, typically 1
	double 		force; // force of the robot
	bool		is_assigned; // is the robot assigned to a task?
};


#endif
