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
#include  <omp.h>
#include <vector>

#include "Object.h"
#include "Coalition.h"
#include "Robot_agent.h"


//enum ENUM_State{Com_Stop,Com_Break, Com_Safe};

//enum ENUM_State_Orie{Not_Follow,Per_Follow};

//enum ENUM_State_of_prediction{Ballistic,Straight};

using namespace Eigen;
//using namespace std;
//double minPos[3] = {-0.5, -1.00, 0.3};
//double maxPos[3] = {0.0,  1.00, 1.0};

//const int N_coordination_allocation_paramerets=2;

const double VALUATION_THRESHOLD = 1;
const int MAX_COALITION_SIZE = 3;

const int MAX_TASKS = 6;
//const int Max_Grabbing_state=4;

const double OBJ_MAX_PREDICTIONTIME = 100;



// test
/*

struct S_Virtual_object {
	bool Grabbing_state_is_set[Max_Grabbing_state];
	int N_grabbing_pos;
	VectorXd X_V_;									// The State of the virtual object
	VectorXd X_V_INTERCEPT_;						// The State of the virtual object with respect to the intercept point
	VectorXd DX_V_INTERCEPT_;						// The D-State of the virtual object
	VectorXd X_V_G_[Max_Grabbing_state];			// The State of the grabbing positions with respect to the virtual object
	VectorXd U_[Max_Grabbing_state];				// The force between the i th robot and the virtual object
	VectorXd U_sum_;								// The force between the i th robot and the virtual object
	double gamma_;									//Coordination parameter
	double Dgamma_;									//Derivative of Coordination parameter
	double tau_sum_;								//sum of Coordination allocation parameter
};
*/

class Task_allocation
{
public:
	Task_allocation();
	Task_allocation(double dt, int n_state, int max_n_bots, int max_n_tasks, MatrixXd A_V,Object_prediction_type Object_motion=Object_prediction_type::Straight);
	~Task_allocation();
//	void 		Initialize(int N_robots, int N_grabbing_pos, double dt, int N_state, MatrixXd A_V,ENUM_State_of_prediction Object_motion=Straight);
//	void 		Initialize_robot(int index,int Num_LPV_Com, const char  *path_A_LPV, const char  *path_prior_LPV,const char  *path_mu_LPV,const char  *path_sigma_LPV
//						  ,int Num_GMM_Com, int Num_GMM_state, const char  *path_prior_GMM,const char  *path_mu_GMM,const char  *path_sigma_GMM, const char *path_threshold,Vector3d X_Base);


	//void 		Initialize_multiple_objects(int N_robots, int N_objects, S_object* Objs_, double dt, int N_state, MatrixXd A_V,ENUM_State_of_prediction Object_motion); // patrick


	int			add_robot(Robot_agent bot);
	int 		add_task(Object Object);

	bool 		Get_prediction_state();
	bool 		Get_catching_state();
	void 		Get_Virtual_state(VectorXd & X);
	void 		Get_the_grabbing_state(int index, VectorXd& X);

	VectorXd	get_object_state(int i);
	void 		predict_motion();

/*	void 		Get_the_desired_intercept_state(int index, VectorXd& X);*/
	void 		Get_the_robot_state(int index, VectorXd& X);
	void 		Get_predict_the_object_position(int index, MatrixXd& X);
	void 		Get_index_of_grabbing_posititon_(int index_of_robot, int& index_of_grabbing, Vector3d& X_I_C);
	bool 		Get_pos_of_grabbing_posititon_for_object_(double& likelihood, Vector3d& X_I_C);


	bool		set_object_state(int i, VectorXd X, VectorXd DX);
	void 		predict_the_object_position();
	void		predict_the_objects_position(); // patrick
	void 		Update();

	double 		coalition_evaluate_task(int coal_size, int coalition_id, int object);
	double 		robot_evaluate_task(int i_robot, int i_object, int frame);
	void		allocate(); // patrick

	int			get_1();
	void		init_coalitions();

private:

	void 	ERROR();
	void	restart_everything();
	void 	calculate_coordination_allocation();
	void 	calculate_coordination_parameter();
	void 	calculate_robot_next_state();
	bool	everythingisreceived();
	void	assign_the_robots();
	void	CalculateCatchingprobability(int nFrame,int i_robot,int i_object);
	void 	calculate_ATX();
	void	calculate_u();
	int 	factorial(int n);

	MatrixXd PermGenerator(int n, int k);

	Object_prediction_type Prediction_model;

	bool				catching_pos_is_found;

	int 				n_state;
	int 				max_n_robots;
	int 				n_robots;
	std::vector<Robot_agent> Robots;
//	Robot_agent*		Robots;


	int 				max_n_objects;
	int 				n_objects; // patrick
//	Object*				Objects;
	std::vector<Object>		Objects;
	double 				dt;

	std::vector< std::vector<Coalition> > 		Coalitions;

	MatrixXd 			A_V;

//	S_Robot_ds 			*Robots;
//	S_object 			Object;
//	S_object*			Objects; // patrick
//	S_Virtual_object	Vobject;

	int 				n_frames; // patrick

};


/*



*/
#endif
