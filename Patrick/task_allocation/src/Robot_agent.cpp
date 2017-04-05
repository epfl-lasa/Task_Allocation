/*
 * Robot_agent.cpp
 *
 *  Created on: 12 mars 2017
 *      Author: Patrick
 */

#include "Robot_agent.h"

//using namespace Eigen; // why do I have to put this here? Odd....



Robot_agent::Robot_agent()
{

}

Robot_agent::Robot_agent(int Num_LPV_Com, const char *path_A_LPV, const char *path_prior_LPV,const char *path_mu_LPV, const char *path_sigma_LPV,
				int Num_GMM_Com, int Num_GMM_state, const char *path_prior_GMM, const char *path_mu_GMM, const char *path_sigma_GMM,
				const char *path_threshold, Vector3d X_Base, int ID, int grip, double force_)
{


	int n_state = 6;

	Workspace_model.initialize(Num_GMM_Com,Num_GMM_state);
	//Dynamic.initialize(Num_LPV_Com,n_state);

	Workspace_model.initialize_GMM(path_prior_GMM,path_mu_GMM,path_sigma_GMM,path_threshold);
	//Dynamic.initialize_theta(path_prior_LPV,path_mu_LPV,path_sigma_LPV);
	//Dynamic.initialize_A(path_A_LPV);
	X_base = X_Base;

	X.resize(n_state); X.setZero();

	workspace_model_is_set = true;
	//LPV_is_set = true;
	//tau = 0.0001;
	//Dtau = 0;

	id = ID;
	n_grippers = grip;
	force = force_;

	assignment = -1;
}


Robot_agent::Robot_agent(GMM model, VectorXd base, Vector3d initial, LPV dyn_mod, VectorXd ATX_,
			VectorXd X_, VectorXd X_intercept_, VectorXd DX_, VectorXd X_I_C_, VectorXd X_F_P_,
			VectorXd DX_F_P_, VectorXd X_d_, VectorXd DX_d_, double tau_, double Dtau_, double DDtau_,
			MatrixXd Prob_of_catching_, int n_grippers_, double force_)
{


	Workspace_model = model;
	workspace_model_is_set = true;

	X_base = base;
	X_initial_pose = initial;
//	Dynamic = dyn_mod;
//	LPV_is_set = true;

//	ATX = ATX_;
	X = X_;
//	X_intercept = X_intercept_;
//	DX = DX_;
//	state_is_set = true;
//	X_I_C = X_I_C_;
//	X_F_P = X_F_P_;
//	DX_F_P = DX_F_P_;


//	X_d = X_d_;
//	DX_d = DX_d_;
//	desired_state_is_set = true;

//	tau = 0;
//	Dtau = 0;
//	DDtau = 0;
//	Probability_of_catching = Prob_of_catching_;

	n_grippers = n_grippers_;
	force = force_;
//	is_assigned = false;

//	index_of_grabbing_position = 0;

//	M = NULL;
}


void Robot_agent::set_assignment(int assignment_)
{
	assignment = assignment_;
}

int Robot_agent::get_assignment() const
{
	return assignment;
}


void Robot_agent::get_state(VectorXd& X_) const
{
	X_ = X;
}


bool Robot_agent::get_workspace_set() const
{
	return workspace_model_is_set;
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
/*	if (state_is_set == true)
	{
		cout<<"States of robot is already  set."<<endl;
//		ERROR();
	}
*/

	X = X_;
//	state_is_set = true;
}

VectorXd Robot_agent::get_base() const
{
	return X_base;
}

int Robot_agent::get_n_grippers() const
{
	return n_grippers;
}

double Robot_agent::get_force() const
{
	return force;
}

int Robot_agent::get_id() const
{
	return id;
}


double Robot_agent::evaluate_task(const Object& obj)
{
	int n_grips = obj.get_n_grippers();
	Vector3d delta[n_grips];
	double delta_norm = 1000000;
	double temp_delta;
	MatrixXd POG[n_grips];
/*	for(int i = 0; i < n_grips; i++)
	{
		POG[i] = obj.get_P_O_G_prediction(i);
	//	cout << " POG " << i << endl << POG[i] << endl;
	//	cout << " POG " << i  << " has " << POG[i].cols() << " columns" << endl;
	//	cout << "last column is " << endl << POG[i].col(POG[i].cols()-1) << endl;
		temp_delta = (X_base.block(0,0,3,1) - (POG[i].col(0)).block(0,0,3,1)).norm();
		if(temp_delta < delta_norm)
			delta_norm = temp_delta;
	}
*/
	temp_delta = (X_base.block(0,0,3,1) - obj.get_X_O().block(0,0,3,1)).norm();
	delta_norm = temp_delta;
	// check feasibility somehow
	//if(delta_norm > 20)
	//	delta_norm = 1000000;
//	cout << "delta norm = " << delta_norm << endl;
	return delta_norm;
}

/*
bool Robot_agent::init_robot(VectorXd base, Vector3d X_init, VectorXd X, VectorXd ATX_, LPV Dynamic, GMM Workspace, int grippers, double force)
{

	set_base(base);
//	set_LPV(Dynamic);
//	set_ATX(ATX_);
//	set_grippers(grippers);
//	set_force(force);
//	set_initial_state(X_init);
	set_state(X);

	return true;
}
*/

void Robot_agent::set_base(Vector3d X)
{
	X_base = X;
}


/*
void Robot_agent::set_grippers(int n)
{
	n_grippers = n;
}

*/

/*
void Robot_agent::set_force(double force_)
{
	force = force_;
}

*/


std::ostream& operator <<(std::ostream& stream, const Robot_agent& o)
{

	cout << "ID " << o.id << endl;
	cout << "workspace set " << o.workspace_model_is_set << endl;
	cout << "X base " << endl << o.X_base << endl;
	cout << "X " << endl << o.X << endl;
	cout << "n grippers " << o.n_grippers << endl;
	cout << "force " << o.force << endl;
	cout << "assigned to task " << o.assignment << endl;

}

/*
void Robot_agent::set_LPV(LPV model)
{
	Dynamic = model;
}

void Robot_agent::set_ATX(VectorXd ATX_)
{
	ATX = ATX_;
}*/


/*
void Robot_agent::set_first_primitive_desired_position(VectorXd X_, VectorXd DX_)
{
	/* Setting the desired state of the first primitive of the  index th robot
		 * X is the desired state of the end-effector with respect to the world-frame
		 * DX is the D-desired state of the end-effector
		 * 	DOES NOT NEED TO CALL AT EACH UPDATE LOOP		*/

/*		if ((X_F_P.rows() != X_.rows()) || (DX_F_P.rows() != DX_.rows()))
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


/*
void Robot_agent::get_coordination_allocation(int index, double& x)
{
	x=tau;
}

void Robot_agent::get_coordination_parameter(double& x)
{
	x=gamma;
}
*/



/*
//	 * X is the state of the end-effector with respect to the world-frame
void Robot_agent::set_initial_state(Vector3d X_)
{



		X_initial_pose = X;
		cout<<"The  initial position of robot is:"<<endl<< X_initial_pose<<endl;
}
*/

/*
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
*/
