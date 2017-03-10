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

#include <stdio.h>
#include <stdlib.h>
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "LPV.h"
#include "TrajectoryEstimation.h"
#include <math.h>
#include  <omp.h>

enum ENUM_State{Com_Stop,Com_Break, Com_Safe};

enum ENUM_State_Orie{Not_Follow,Per_Follow};

enum ENUM_State_of_prediction{Ballistic,Straight};

using namespace Eigen;
double minPos[3] = {-0.5, -1.00, 0.3};
double maxPos[3] = {0.0,  1.00, 1.0};

const int N_coordination_allocation_paramerets=2;

const double VALUATION_THRESHOLD = 1;
const int MAX_COALITION_SIZE = 3;


const int Max_Grabbing_state=4;

const double OBJ_MAX_PREDICTIONTIME = 100;


struct S_Coalition {
	int n_robots;   // number of robots
	int* robots_id; // IDs of the robots so we can adress them later

	int n_grippers; // available grippers

	double* values; // tasks values
	double coalitional_value;


	int n_grippers_req; // required grippers
	double force; // required force
};



struct S_Robot_ds {
	bool Workspace_model_is_set_;
	bool the_state_is_set_;
	bool the_LPV_is_set_;
	bool the_desired_state_is_set_;
	int 		index_of_grabbing_posititon_;
	GMM 		Workspace_model_;
	VectorXd 	X_Base_; 			//Position of the base of the robot with respect to the world frame
	Vector3d 	X_Initial_pose_; 	// Initial position of the end-effector of the robot with respect to the world frame
	LPV			Dynamic_;
	VectorXd 	ATX_;		//To simplify the calculations!
	VectorXd 	X_; 					//State of the robot in the world frame
	VectorXd 	X_INTERCEPT_; 			//State of the robot in the world frame
	VectorXd 	DX_;
	VectorXd 	X_I_C_;		// Desired intercept point of the i th robot
	VectorXd 	X_F_P_;		// First primitive desired state
	VectorXd 	DX_F_P_;	// First primitive desired D-state
	VectorXd 	X_d_;		//The desired state of the robot
	VectorXd 	DX_d_;		//The derivative of the desired state of the robot
	double 		tau_;		//Coordination allocation
	double		Dtau_;		//Derivative of Coordination allocation
	double		DDtau_;		//Derivative of Coordination allocation
	MatrixXd	Probability_of_catching_;
	double 		M[N_coordination_allocation_paramerets];


	int 		n_grippers; // number of grippers on this robot, typically 1
	double 		force; // force of the robot
	bool		is_assigned; // is the robot assigned to a task?
};


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


class task_allocation
{
public:

	void 		Initialize(int N_robots, int N_grabbing_pos, double dt, int N_state, MatrixXd A_V,ENUM_State_of_prediction Object_motion=Straight);
	void 		Initialize_robot(int index,int Num_LPV_Com, const char  *path_A_LPV, const char  *path_prior_LPV,const char  *path_mu_LPV,const char  *path_sigma_LPV
						  ,int Num_GMM_Com, int Num_GMM_state, const char  *path_prior_GMM,const char  *path_mu_GMM,const char  *path_sigma_GMM, const char *path_threshold,Vector3d X_Base);


	void 		Initialize_multiple_objects(int N_robots, int N_objects, S_object* Objs_, double dt, int N_state, MatrixXd A_V,ENUM_State_of_prediction Object_motion); // patrick
	void 		Set_the_initial_robot_state(int index,Vector3d X);
	void 		Set_the_robot_state(int index,VectorXd X);
	void 		Set_the_robot_first_primitive_desired_position(int index,VectorXd X,VectorXd DX);

	bool 		Get_prediction_state();
	bool 		Get_catching_state();
	void 		Get_Virtual_state(VectorXd & X);
	void 		Get_the_grabbing_state(int index, VectorXd& X);
	void 		Get_the_coordination_allocation(int index, double& x);
	void 		Get_the_coordination_parameter(double& x);
/*	void 		Get_the_desired_intercept_state(int index, VectorXd& X);*/
	void 		Get_the_robot_state(int index, VectorXd& X);
	void 		Get_predict_the_object_position(int index, MatrixXd& X);
	void 		Get_index_of_grabbing_posititon_(int index_of_robot, int& index_of_grabbing, Vector3d& X_I_C);
	bool 		Get_pos_of_grabbing_posititon_for_object_(double& likelihood, Vector3d& X_I_C);


	void 		predict_the_object_position();
	void		predict_the_objects_position(); // patrick
	void 		Update();

	double 		coalition_evaluate_task(int coal_size, int coalition_id, int object);
	double 		robot_evaluate_task(int i_robot, int i_object, int frame);
	void		allocate(); // patrick
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

	bool				The_catching_pos_is_found;
	int 				N_robots_;
	int 				N_state_;
	int 				N_objects_; // patrick
	double 				dt_;

	VectorXd		 	X_V_;
	VectorXd 			DX_V_;

	MatrixXd 			A_V_;

	S_Robot_ds 			*Robots_;
	S_object 			Object_;
	S_object*			Objects_; // patrick
	S_Virtual_object	Vobject_;

	int 				N_frames_; // patrick

	S_Coalition**		Coalitions_; // patrick set of all possible coalitions, many will be of value 0


	double				handle_exp_old;



};




int factorial(int n)
{
    if(n > 1)
        return n * factorial(n - 1);
    else
        return 1;
}
MatrixXd PermGenerator(int n, int k)
{
	MatrixXd handle(factorial(n)/factorial(n-k),k);
    std::vector<int> d(n);
    std::iota(d.begin(),d.end(),1);
    cout << "These are the Possible Permutations: " << endl;
    int repeat = factorial(n-k);
    int counter=0;
    do
    {
        for (int i = 0; i < k; i++)
        {
        	handle(counter,i)=d[i]-1;
        }
        counter=counter+1;
        for (int i=1; i!=repeat; ++i)
        {
            next_permutation(d.begin(),d.end());
        }
    } while (next_permutation(d.begin(),d.end()));

    cout<<handle<<endl;

    return handle;
}
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
