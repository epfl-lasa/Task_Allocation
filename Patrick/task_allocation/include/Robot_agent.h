/*
 * Robot_agent.h
 *
 *  Created on: 12 mars 2017
 *      Author: Patrick
 */


/* TODO
 * add the functions from multiarm_ds that handle robots in here
 */


#ifndef ROBOT_AGENT_H_
#define ROBOT_AGENT_H_


#include <stdio.h>
#include <stdlib.h>
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "LPV.h"
#include <math.h>
#include  <omp.h>
#include "Object.h"

using namespace Eigen;

class Robot_agent {

public:
	Robot_agent();
	Robot_agent(int Num_LPV_Com, const char  *path_A_LPV, const char  *path_prior_LPV,const char  *path_mu_LPV,const char  *path_sigma_LPV, int Num_GMM_Com,
					int Num_GMM_state, const char  *path_prior_GMM,const char  *path_mu_GMM,const char  *path_sigma_GMM, const char *path_threshold, Vector3d X_Base, int ID, int grip, double force);


	double evaluate_task(const Object& obj);
	VectorXd compute_intercept(const Object& obj);


	void set_idle_target();
	void set_base(const geometry_msgs::Pose & msg);
	void set_end(const geometry_msgs::Pose & msg);

	void set_state(VectorXd X);
	void set_assignment(int);


	bool get_state_set() const;
	void get_state(VectorXd& X) const;
	VectorXd get_base() const;
	int get_n_grippers() const;
	double get_force() const;
	int	get_id() const;
	int get_assignment() const;
	VectorXd get_intercept() const;
	Vector3d get_end() const;

	friend std::ostream& operator <<(std::ostream& stream, const Robot_agent& o);
private:

	GMM 		Workspace_model;
	Vector3d 	X_base; 			//Position of the base of the robot with respect to the world frame
	Vector3d	X_end;
	Vector3d 	X_initial_pose; 	// Initial position of the end-effector of the robot with respect to the world frame
//	LPV			Dynamic;
	VectorXd 	X; 					//State of the robot in the world frame
	VectorXd	X_targ;				// target of the robot

	int 		id;
	int 		n_grippers; // number of grippers on this robot, typically 1
	double 		force; // force of the robot
	int 		assignment;


};


#endif
