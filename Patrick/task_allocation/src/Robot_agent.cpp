/*
 * Robot_agent.cpp
 *
 *  Created on: 12 mars 2017
 *      Author: Patrick
 */

#include "Robot_agent.h"

using namespace Eigen; // why do I have to put this here? Odd....



Robot_agent::Robot_agent()
{

}

Robot_agent::Robot_agent(GMM model, VectorXd base, Vector3d initial, LPV dyn_mod, VectorXd ATX_,
			VectorXd X_, VectorXd X_intercept_, VectorXd DX_, VectorXd X_I_C_, VectorXd X_F_P_,
			VectorXd DX_F_P_, VectorXd X_d_, VectorXd DX_d_, double tau_, double Dtau_, double DDtau_,
			MatrixXd Prob_of_catching_, int n_grippers_, double force_)
{
/*
 * bool workspace_model_is_set;
	bool state_is_set;
	bool LPV_is_set;
	bool desired_state_is_set;
	int 		index_of_grabbing_posititon;
	GMM 		Workspace_model;
	VectorXd 	X_Base; 			//Position of the base of the robot with respect to the world frame
	Vector3d 	X_initial_pose; 	// Initial position of the end-effector of the robot with respect to the world frame
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
 */


	Workspace_model = model;
	workspace_model_is_set = true;

	X_base = base;
	X_initial_pose = initial;
	Dynamic = dyn_mod;
	LPV_is_set = true;

	ATX = ATX_;
	X = X_;
	X_intercept = X_intercept_;
	DX = DX_;
	state_is_set = true;
	X_I_C = X_I_C_;
	X_F_P = X_F_P_;
	DX_F_P = DX_F_P_;


	X_d = X_d_;
	DX_d = DX_d_;
	desired_state_is_set = true;

	tau = 0;
	Dtau = 0;
	DDtau = 0;
	Probability_of_catching = Prob_of_catching_;

	n_grippers = n_grippers_;
	force = force_;
	is_assigned = false;

	index_of_grabbing_position = 0;

	M = NULL;
}

Robot_agent::~Robot_agent()
{
	if(M != NULL)
		delete M;
}


bool Robot_agent::get_state_set()
{
	return state_is_set;
}

bool Robot_agent::get_LPV_set()
{
	return LPV_is_set;
}

bool Robot_agent::get_desired_state_set()
{
	return desired_state_is_set;
}

void Robot_agent::get_state(VectorXd& X_)
{
	X_ = X;
}


bool Robot_agent::get_workspace_set()
{
	return workspace_model_is_set;
}

void Robot_agent::get_coordination_allocation(int index, double& x)
{
	x=tau;
}

void Robot_agent::get_coordination_parameter(double& x)
{
	x=gamma;
}






void Robot_agent::set_initial_state(Vector3d X_)
{
	/*
		 * X is the state of the end-effector with respect to the world-frame
		 * 								*/
		X_initial_pose = X;
		cout<<"The  initial position of robot is:"<<endl<< X_initial_pose<<endl;
}

void Robot_agent::set_state(VectorXd X_)
{
	/* Setting the current state
	 * X is the state of the end-effector with respect to the world-frame
	 * 								*/
	if (X.rows() != X_.rows())
	{
		cout<<"The state dimension of robot is wrong."<<endl;
		cout<<"The dimension of X is "<<X_.rows()<<endl;
		cout<<"The dimension of robot is "<<X.rows()<<endl;
//		ERROR();
	}
	if (state_is_set == true)
	{
		cout<<"States of robot is already  set."<<endl;
//		ERROR();
	}

	X = X_;
	state_is_set = true;
}


void Robot_agent::set_first_primitive_desired_position(VectorXd X_, VectorXd DX_)
{
	/* Setting the desired state of the first primitive of the  index th robot
		 * X is the desired state of the end-effector with respect to the world-frame
		 * DX is the D-desired state of the end-effector
		 * 	DOES NOT NEED TO CALL AT EACH UPDATE LOOP		*/

		if ((X_F_P.rows() != X_.rows()) || (DX_F_P.rows() != DX_.rows()))
		{
			cout<<"The dimension of the desired state of robot is wrong."<<endl;
			cout<<"The dimension of X is "<<X_.rows()<<" and "<<DX_.rows()<<endl;
			cout<<"The dimension of robot is "<<X_F_P.rows()<<endl;
		//	ERROR();
		}


		X_F_P = X_;
		DX_F_P = DX_;
		desired_state_is_set = true;
}

int Robot_agent::get_n_grippers()
{
	return n_grippers;
}

double Robot_agent::get_force()
{
	return force;
}
