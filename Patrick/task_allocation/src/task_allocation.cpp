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


#include "task_allocation.h"


// patrick
void task_allocation::Initialize_multiple_objects(int N_robots, int N_objects, S_object* Objects, double dt, int N_state, MatrixXd A_V,ENUM_State_of_prediction Object_motion)
{
	N_robots_ = N_robots;
	N_state_= N_state;
	N_objects_ = N_objects;
	dt_ = dt;
	Robots_= new S_Robot_ds[N_robots_];
	Objects_ = new S_object[N_objects_];
	for(int i = 0; i < N_objects_; i++)
	{
		Objects_[i] = Objects[i];
	}

	// initialize the robots.



	// initialize coalitions
	// allocate a large double array that holds all possible coalitions... Might duck up here
	// Robots have not yet been initialized at this point btw...
	Coalitions_ = new S_Coalition*[MAX_COALITION_SIZE];
	for(int i = 0; i < MAX_COALITION_SIZE; i++)
	{
		MatrixXd coals = PermGenerator(N_robots_, i);
		int n_coals = coals.rows();
		Coalitions_[i] = new S_Coalition[n_coals];

		for(int j = 0; j < n_coals; j++)
		{
			// init coalition with size and set force, grippers and value to 0
			Coalitions_[i][j].n_robots = i;
			Coalitions_[i][j].robots_id = new int[Coalitions_[i][j].n_robots];
			Coalitions_[i][j].force = 0;
			Coalitions_[i][j].n_grippers = 0;
			Coalitions_[i][j].coalitional_value = 0;

			// note all robot ids in the coalition and sum the grippers and force.
			for(int k = 0; k < i; k++)
			{
				Coalitions_[i][j].robots_id[k] = coals(j,k);
				Coalitions_[i][j].force += Robots_[Coalitions_[i][j].robots_id[k]].force;
				Coalitions_[i][j].n_grippers += Robots_[Coalitions_[i][j].robots_id[k]].n_grippers;
			}
		}
	}


	// not sure what this is for
	A_V_.resize(N_state_,N_state_);				A_V_.setZero();
	X_V_.resize(N_state_);						X_V_.setZero();
	DX_V_.resize(N_state_);						DX_V_.setZero();



	// init robots
	for (int i=0;i<N_robots_;i++)
	{
		Robots_[i].X_.resize(N_state_);							Robots_[i].X_.setZero();
		Robots_[i].DX_.resize(N_state_);						Robots_[i].DX_.setZero();
		Robots_[i].X_d_.resize(N_state_);						Robots_[i].X_d_.setZero();
		Robots_[i].DX_d_.resize(N_state_);						Robots_[i].DX_d_.setZero();
		Robots_[i].X_F_P_.resize(N_state_);						Robots_[i].X_F_P_.setZero();
		Robots_[i].DX_F_P_.resize(N_state_);					Robots_[i].DX_F_P_.setZero();
		Robots_[i].ATX_.resize(N_state_);						Robots_[i].ATX_.setZero();
		Robots_[i].X_I_C_.resize(N_state_);						Robots_[i].X_I_C_.setZero();
		Robots_[i].Workspace_model_is_set_=false;
		Robots_[i].the_state_is_set_=false;
		Robots_[i].the_LPV_is_set_=false;
		Robots_[i].the_desired_state_is_set_=false;
	}


}

double task_allocation::coalition_evaluate_task(int coal_size, int coalition_id, int object)
{

	double evaluation = 0;
	// if coalition can't do the task due to not enough grippers, set to 0.
	// Bold move: if coalition has too many grippers, set to 0.
	if(Coalitions_[coal_size][coalition_id].n_grippers != Objects_[object].N_grabbing_pos_)
	{
		evaluation = 0;
	}
	/*else
	{
		for(int i = 0; i < coal_size; i++)
		{
			evaluation += robot_evaluate_task(Coalitions_[coal_size][coalition_id].robots_id[i], object, 0);
		}
	}*/

	else
	{
		evaluation = Objects_[object].value - (Coalitions_[coal_size][coalition_id].force - Objects_[object].weight); // + Objects_[object].weight
	}
	return evaluation;
}

void task_allocation::Initialize(int N_robots,int N_grabbing_pos, double dt, int N_state, MatrixXd A_V,ENUM_State_of_prediction Object_motion)
{
	/* Declare the number of the robots
	 * N_grabbing_pos is the number of the grabbing position.
	 * sample time (dt)
	 * the dimension  of the state
	 * and the gain matrix of the virtual object   */

	if (N_grabbing_pos>N_grabbing_pos)
	{
		cout<<"Number of the grabbing positions is more than the available robots."<<endl;
		ERROR();
	}
	Object_.order_of_grabbing_.resize(factorial(N_robots)/factorial(N_robots-N_grabbing_pos),N_grabbing_pos);
	Object_.order_of_grabbing_=PermGenerator(N_robots,N_robots);


	N_robots_=N_robots;
	N_state_=N_state;
	dt_=dt;
	Robots_=new S_Robot_ds[N_robots_];

	A_V_.resize(N_state_,N_state_);				A_V_.setZero();
	X_V_.resize(N_state_);						X_V_.setZero();
	DX_V_.resize(N_state_);						DX_V_.setZero();

	Object_.X_O_.resize(N_state_);				Object_.X_O_.setZero();
	Object_.X_O_First_.resize(N_state_);		Object_.X_O_First_.setZero();
	Object_.DX_O_.resize(N_state_);				Object_.DX_O_.setZero();
	Object_.X_I_C_.resize(N_state_);			Object_.X_I_C_.setZero();
	Object_.X_O_INTERCEPT_.resize(N_state_);	Object_.X_O_INTERCEPT_.setConstant(10);

	Object_.MAX_PREDICTIONTIME_ = OBJ_MAX_PREDICTIONTIME;

	Vobject_.X_V_.resize(N_state);				Vobject_.X_V_.setZero();
	Vobject_.DX_V_INTERCEPT_.resize(N_state);	Vobject_.DX_V_INTERCEPT_.setZero();

	Object_.P_O_prediction_.resize((int)floor(Object_.MAX_PREDICTIONTIME_/(dt_))+1,3);
	for (int i=0;i<N_grabbing_pos;i++)
	{
		Vobject_.X_V_G_[i].resize(N_state);		Vobject_.X_V_G_[i].setZero();
		Vobject_.U_[i].resize(N_state);			Vobject_.U_[i].setZero();
		Vobject_.U_sum_.resize(N_state);		Vobject_.U_sum_.setZero();
		Object_.X_O_G_[i].resize(N_state);		Object_.X_O_G_[i].setZero();
		Object_.P_O_G_prediction_[i].resize((int)floor(Object_.MAX_PREDICTIONTIME_/(100*dt_))+1,3);
		Object_.P_O_G_prediction_[i].setZero();
	}
	for (int i=0;i<N_robots_;i++)
	{
		Robots_[i].X_.resize(N_state_);							Robots_[i].X_.setZero();
		Robots_[i].DX_.resize(N_state_);						Robots_[i].DX_.setZero();
		Robots_[i].X_d_.resize(N_state_);						Robots_[i].X_d_.setZero();
		Robots_[i].DX_d_.resize(N_state_);						Robots_[i].DX_d_.setZero();
		Robots_[i].X_F_P_.resize(N_state_);						Robots_[i].X_F_P_.setZero();
		Robots_[i].DX_F_P_.resize(N_state_);					Robots_[i].DX_F_P_.setZero();
		Robots_[i].ATX_.resize(N_state_);						Robots_[i].ATX_.setZero();
		Robots_[i].X_I_C_.resize(N_state_);						Robots_[i].X_I_C_.setZero();
		Robots_[i].Workspace_model_is_set_=false;
		Robots_[i].the_state_is_set_=false;
		Robots_[i].the_LPV_is_set_=false;
		Robots_[i].the_desired_state_is_set_=false;
	}

	if ((A_V.rows()!=N_state_)||(A_V.cols()!=N_state_))
	{
		cout<<"Initialization of "<<A_V<<" is wrong."<<endl;
		cout<<"A_V: "<<endl;cout<<A_V<<endl;
		ERROR();
	}
	A_V_=A_V;

	if (N_grabbing_pos>Max_Grabbing_state)
	{
		cout<<"The number of the grabbing state is higher than the Max. Just got to task_allocation.h and change Max_Grabbing_state"<<endl;
		ERROR();
	}
	Object_.N_grabbing_pos_=N_grabbing_pos;

	for (int i=0;i<Object_.N_grabbing_pos_;i++)
	{
		Object_.Grabbing_state_is_set_[i]=false;
		Vobject_.Grabbing_state_is_set[i]=false;
	}


	double gravity[3];gravity[0]=0;gravity[1]=0;
	Object_.Object_state_is_set_=false;
	Object_.Object_motion_=Object_motion;
	if (Object_.Object_motion_==Ballistic)
	{
		gravity[2]=9.81;
	}
	else
	{
		gravity[2]=0;
	}

	Object_.First_Object_state_is_set_=false;
	Object_.predict_ = new TrajectoryEstimation(gravity, dt_,30,10);
	cout<<"Object_.predict  is done"<<endl;
	Matrix3d handle;handle.setIdentity();
	Object_.predict_->setMomentOfInertia(handle);
	Object_.predict_->resetTrajectoryEstimation();
	The_catching_pos_is_found=false;
	Object_.Max_liklihood=0;
}

void task_allocation::Initialize_robot(int index,int Num_LPV_Com, const char  *path_A_LPV, const char  *path_prior_LPV,const char  *path_mu_LPV,const char  *path_sigma_LPV,int Num_GMM_Com, int Num_GMM_state, const char  *path_prior_GMM,const char  *path_mu_GMM,const char  *path_sigma_GMM, const char *path_threshold,Vector3d X_Base)
{
	/*Declare the parameters of   index th robot.
	 * Num_LPV_Com Number of components of LPV
	 * path_A_LPV is the path to A matrices
	 * path_prior_LPV is the prior of Theta function
	 * path_mu_LPV is the Mean of Theta function
	 * path_sigma_LPV is the Sigma of Theta function
	 * Num_GMM_Com	 is the number of the components of the robot's workspace model
	 * Num_GMM_state is the number of the states of the robot's workspace model
	 * Num_GMM_state is the number of the states of the robot's workspace model
	 * path_prior_GMM is the number of the states of the robot's workspace model
	 * path_mu_GMM is the number of the states of the robot's workspace model
	 * path_sigma_GMM is the number of the states of the robot's workspace model
	 * X_Base is the position of the base of the robot*/



	if (index>N_robots_-1)
	{
		cout<<"Initialization of "<<index<<"th robot is wrong."<<endl;
		cout<<"index "<<index<<" Max robot Number "<<N_robots_-1<<endl;
		ERROR();
	}


	if (Robots_[index].Workspace_model_is_set_||Robots_[index].the_LPV_is_set_)
	{
		cout<<"Initialization of "<<index<<"th robot is wrong."<<endl;
		cout<<"This robot is already being initialized"<<endl;
		ERROR();
	}
	if (X_Base.rows()!=3)
	{
		cout<<"Initialization of "<<index<<"th robot is wrong."<<endl;
		cout<<"The base of the robot is wrong. The size of X_Base is "<<X_Base.rows()<<endl;
		ERROR();
	}



	Robots_[index].Workspace_model_.initialize(Num_GMM_Com,Num_GMM_state);
	Robots_[index].Dynamic_.initialize(Num_LPV_Com,N_state_);

	Robots_[index].Workspace_model_.initialize_GMM(path_prior_GMM,path_mu_GMM,path_sigma_GMM,path_threshold);
	Robots_[index].Dynamic_.initialize_theta(path_prior_LPV,path_mu_LPV,path_sigma_LPV);
	Robots_[index].Dynamic_.initialize_A(path_A_LPV);
	Robots_[index].X_Base_=X_Base;

	cout<<"The base of "<<index<<"th robot is "<<endl;
	cout<<Robots_[index].X_Base_<<endl;


	Robots_[index].Workspace_model_is_set_=true;
	Robots_[index].the_LPV_is_set_=true;
	Robots_[index].tau_=0.0001;
	Robots_[index].Dtau_=0;
}
void task_allocation::Set_the_robot_state(int index,VectorXd X)
{
	/* Setting the current state of the  index th robot
	 * X is the state of the end-effector with respect to the world-frame
	 * 								*/
	if ((Robots_[index].X_.rows()!=X.rows()))
	{
		cout<<"The state dimension of "<<index<<"th robot is wrong."<<endl;
		cout<<"The dimension of X is "<<X.rows()<<endl;
		cout<<"The dimension of robot is "<<Robots_[index].X_.rows()<<endl;
		ERROR();
	}
	if (Robots_[index].the_state_is_set_==true)
	{
		cout<<"States of "<<index<<"th robot is already being set."<<endl;
		ERROR();
	}

	Robots_[index].X_=X;
	Robots_[index].the_state_is_set_=true;
}
void task_allocation::Set_the_initial_robot_state(int index,Vector3d X)
{
	/* Setting the initial state of the  index th robot
	 * X is the state of the end-effector with respect to the world-frame
	 * 								*/

	Robots_[index].X_Initial_pose_=X;
	cout<<"The  initial position of "<<index<<"th robot is:"<<endl<<Robots_[index].X_Initial_pose_<<endl;
}


void task_allocation::Set_the_robot_first_primitive_desired_position(int index,VectorXd X,VectorXd DX)
{

	/* Setting the desired state of the first primitive of the  index th robot
	 * X is the desired state of the end-effector with respect to the world-frame
	 * DX is the D-desired state of the end-effector
	 * 	DOES NOT NEED TO CALL AT EACH UPDATE LOOP		*/

	if ((Robots_[index].X_F_P_.rows()!=X.rows())||(Robots_[index].DX_F_P_.rows()!=DX.rows()))
	{
		cout<<"The dimension of the desired state of "<<index<<"th robot is wrong."<<endl;
		cout<<"The dimension of X is "<<X.rows()<<" and "<<DX.rows()<<endl;
		cout<<"The dimension of robot is "<<Robots_[index].X_F_P_.rows()<<endl;
		ERROR();
	}


	Robots_[index].X_F_P_=X;
	Robots_[index].DX_F_P_=DX;
	Robots_[index].the_desired_state_is_set_=true;
}


// patrick
void task_allocation::predict_the_objects_position()
{

	for(int i = 0; i < N_objects_; i++)
	{
		if ((Objects_[i].predict_->mReadyToPredict)&&(Objects_[i].X_O_(0)<4*minPos[0]))
		{

			N_frames_ =0;

			// predict the position of center of gravity
			N_frames_ = Objects_[i].predict_->PredictNextPosVel(Objects_[i].MAX_PREDICTIONTIME_,dt_, Objects_[i].P_O_prediction_,true,0,maxPos[0]);

			if 	(N_frames_==-1)
			{
				// no prediction for some reason
				cout << "no prediction available" << endl;
			}
			else
			{
				// predict the positions of each grabbing position
				for (int j=0;j<Object_.N_grabbing_pos_;j++)
				{
					for (int k=0;k<N_frames_;k++)
					{
						Object_.P_O_G_prediction_[j].row(k)=Object_.P_O_prediction_.row(k)+Object_.X_O_G_[j].block(0,0,3,1).transpose();
					}
					//	cout<<"Object_.P_O_G_prediction_[i] "<<i<<" "<<Object_.P_O_G_prediction_[i]<<endl;
				}
			}
		}
	}
}

// returns a double, valuation of the task i_object by robot i_robot.
// no evaluation of each grabbing point
// patrick
double task_allocation::robot_evaluate_task(int i_robot, int i_object, int frame)
{
	double value = 0;
	VectorXd Conveyor_end;
	Conveyor_end.resize(N_state_); Conveyor_end.setConstant(0,0,0);

	// check if indices within bounds
	if(i_robot >= N_robots_ || i_object >= N_objects_)
	{
		cout << "i_Robot > N_robots" << (i_robot >= N_robots_)  << " i_object > N_objects " << (i_object >= N_objects_ ) << endl;
	}
	else
	{
		// compute value of the object, a weighted sum of the distance to the robot + the inverse of the distance to the end of the conveyor
		VectorXd delta_robot = Robots_[i_robot].X_Base_ - Objects_[i_object].P_O_prediction_.row(frame);
		double delta_robot_norm = delta_robot.norm();
		if(delta_robot_norm < VALUATION_THRESHOLD) // if out of reach, then we value at 0
		{
			VectorXd delta_conveyor = Conveyor_end - Objects_[i_object].P_O_prediction_.row(frame);
			value = 20.0*delta_robot.norm();// + 200.0/min(delta_conveyor.norm(), 0.0001) ; // patrick, min is ugly way to avoid dividing by 0
		}
	}


	return value;
}


// patrick
void task_allocation::allocate()
{
	// predict objects
	predict_the_objects_position();

	// do some check to see if prediction worked


	// initialize/reset a few things...
	for(int k = 0; k < N_robots_; k++)
	{
		Robots_[k].is_assigned = false;
	}

	for(int j = 0; j < N_objects_; j++)
	{
		Objects_[j].coalition.coalitional_value = 0;
		Objects_[j].coalition.force = 0;
		Objects_[j].coalition.n_grippers = 0;
		Objects_[j].coalition.n_robots = 0;
		Objects_[j].coalition.robots_id = NULL;
	}



	for(int i = 0; i < MAX_COALITION_SIZE; i++)
	{
		// this is a sub-optimal way of getting the number of coalitions of this dimension
		MatrixXd coals = PermGenerator(N_robots_, MAX_COALITION_SIZE);
		int n_coals = coals.rows();
		for(int j = 0; j < n_coals; j++)
		{
			// free the previously allocated memory, this allows to handle object removal
			if(Coalitions_[i][j].values != NULL)
			{
				delete Coalitions_[i][j].values;
				Coalitions_[i][j].values = NULL;
			}

			Coalitions_[i][j].values = new double[N_objects_];

			// evaluate all tasks for this coalition and find best value
			double best_value = 0;
			double temp_value = 0;
			double best_index = 0;
			for(int k = 0; k < N_objects_; k++)
			{
				temp_value = Coalitions_[i][j].values[k] = coalition_evaluate_task(i,j,k);
				if(temp_value > best_value)
				{
					best_index = k;
					best_value = temp_value;
				}
			}

			// best value is found, if it's non-zero we have the coalitional value
			Coalitions_[i][j].coalitional_value = (best_value > 0)? best_value : 0;

		}
	}





/*
	// compute all valuations
	double robot_valuations[N_objects_][N_robots_]; // could be declared somewhere else, to avoid allocating all the time

	for(int j = 0; j < N_objects_; j++)
	{
		for(int k = 0; k < N_robots_; k++)
		{
			robot_valuations[j][k] = robot_evaluate_task(k,j,N_frames_-1); // get the value of the next prediction step
		}
	}



	// for each object that requires coalition, make them. This doesn't seem to work.
	for(int j = 0; j < N_objects_; j++)
	{
		if(Objects_[j].N_grabbing_pos_ > 1)
		{
			cout << "coalition of multiple robots required for object " << j << " with " << Objects_[j].N_grabbing_pos_ << " grabbing positions." << endl;

			int available_robots = 0;
			int available_id[N_robots_];
			double max_value = 0;
			int best_coalition = -1;
			for(int i = 0; i < N_robots_; i++)
			{
				available_id[i] = -1;
			}

			// find available robots
			cout << "finding available robots" << endl;
			for(int k = 0; k < N_robots_; k++)
			{
				if(robot_valuations[j][k] > 0)
				{
					available_id[available_robots] = k;
					available_robots++;
				}
			}

			// compute all permutations of "required robots" out of "available robots"
			cout << "computing permutations" << endl;
			MatrixXd permutations = PermGenerator(Objects_[j].N_grabbing_pos_, available_robots);

			// compute all coalition values
			S_Coalition coalitions[permutations.rows()];
			for(int i = 0; i < permutations.rows(); i++)
			{
				coalitions[i].value = 0;
				coalitions[i].robots_id = new int[permutations.rows()];
				for(int k = 0; k < permutations.row(i).cols() ; k++ )
				{
					coalitions[i].robots_id[k] = available_id[permutations(i,k)];
					coalitions[i].value += robot_valuations[j][coalitions[i].robots_id[k]];
				}
				if(coalitions[i].value > max_value)
					best_coalition = i;
			}

		}
		else
		{
				// only one robot is needed
		}
	}*/

}





void task_allocation::Update()
{


	if (!everythingisreceived())
	{
		cout<<"You forgot to set sth! and you called Update"<<endl;
		ERROR();
	}
	if (The_catching_pos_is_found)
	{
		calculate_coordination_parameter();
		calculate_coordination_allocation();
		assign_the_robots();
		calculate_ATX();
		calculate_u();


		Object_.X_O_INTERCEPT_=Object_.X_O_-Object_.X_I_C_;
		Vobject_.X_V_INTERCEPT_=Vobject_.X_V_-Object_.X_I_C_;

		Vobject_.DX_V_INTERCEPT_=(Vobject_.gamma_*Object_.DX_O_+Vobject_.Dgamma_*Object_.X_O_INTERCEPT_
				+A_V_*(Vobject_.X_V_INTERCEPT_-Vobject_.gamma_*Object_.X_O_INTERCEPT_)
				+Vobject_.U_sum_)/(1+Vobject_.tau_sum_);

		/*		Vobject_.DX_V_INTERCEPT_=(Vobject_.Dgamma_*Object_.X_O_INTERCEPT_
					+A_V_*(Vobject_.X_V_INTERCEPT_-Vobject_.gamma_*Object_.X_O_INTERCEPT_)
					+Vobject_.U_sum_)/(1+Vobject_.tau_sum_);*/


		calculate_robot_next_state();



		Vobject_.X_V_=Vobject_.X_V_+Vobject_.DX_V_INTERCEPT_*dt_;

		for (int i=0;i<N_robots_;i++)
		{
			Robots_[i].X_=Robots_[i].X_+Robots_[i].DX_*dt_;
		}

	}


	restart_everything();
}


bool task_allocation::Get_prediction_state()
{
	return Object_.predict_->mReadyToPredict;
}

bool task_allocation::Get_catching_state()
{
	return The_catching_pos_is_found;
}
void task_allocation::Get_the_coordination_allocation(int index, double& x)
{
	x=Robots_[index].tau_;
}
void task_allocation::Get_the_coordination_parameter(double& x)
{
	x=Vobject_.gamma_;
}
void task_allocation::Get_predict_the_object_position(int index, MatrixXd& X)
{
	//X.resize(Object_.P_O_G_prediction_[index].rows(),Object_.P_O_G_prediction_[index].cols());
	X=Object_.P_O_G_prediction_[index];
}
void task_allocation::Get_the_robot_state(int index, VectorXd& X)
{

	/* Getting the desired state of index th robot
	 *
	 * X is the desired state of the robot	*/

	X=Robots_[index].X_;
}
void task_allocation::Get_Virtual_state(VectorXd & X)
{
	/* Getting the current state of the virtual object
	 * X is the state of the virtual object	*/

	X=Vobject_.X_V_;
}
void task_allocation::Get_the_grabbing_state(int index, VectorXd & X)
{
	/* Getting the current state of index th grabbing position on the virtual object with respect to the world frame
	 * X is the state of the virtual object	*/

	X=Vobject_.X_V_G_[index]+Vobject_.X_V_;
}
bool task_allocation::Get_pos_of_grabbing_posititon_for_object_(double& likelihood, Vector3d& X_I_C)
{
	/* Getting the desired grabbing position of the virtual object
	 * X_I_C is the desired grabbing position in world frame
	 * likelihood is the related to change to grab!	*/
	if (The_catching_pos_is_found)
	{
		X_I_C=Object_.X_I_C_.block(0,0,3,1);
		likelihood=Object_.Max_liklihood;
	}
	return The_catching_pos_is_found;
}
void task_allocation::Get_index_of_grabbing_posititon_(int index_of_robot, int& index_of_grabbing, Vector3d& X_I_C)
{

	/* Getting the desired grabbing position of the object for index_of_robot th robot with respect to the world frame
	 * X_I_C is the desired grabbing position in world frame
	 * index_of_grabbing the index of the desired grabbing position	*/


	if (The_catching_pos_is_found)
	{
		index_of_grabbing=Robots_[index_of_robot].index_of_grabbing_posititon_;
		X_I_C=Robots_[index_of_robot].X_I_C_.block(0,0,3,1);
	}


}


/*void task_allocation::Get_the_desired_intercept_state(int index, VectorXd & X)
{
	 Getting the desired intercept point of index th robot
 * X is the desired intercept point

	X=Robots_[index].X_I_C_.block(0,0,3,1);
}*/




















void task_allocation::ERROR()
{
	while(ros::ok())
	{

	}
}
void task_allocation::restart_everything()
{
	for(int i=0;i<N_robots_;i++)
	{
		Robots_[i].the_state_is_set_=false;
	}
	Object_.Object_state_is_set_=false;

}
bool task_allocation::everythingisreceived()
{
	bool flag=true;

	for (int i=0;i<N_robots_;i++)
	{
		if ((Robots_[i].the_state_is_set_==false)||(Robots_[i].Workspace_model_is_set_==false)||(Robots_[i].the_LPV_is_set_==false)||(Robots_[i].the_desired_state_is_set_==false))
		{
			flag=false;
			cout<<"Sth from robot is missing"<<endl;
			cout<<"Robots_[i].the_state_is_set "<<Robots_[i].the_state_is_set_<<endl;
			cout<<"Robots_[i].Workspace_model_is_set_ "<<Robots_[i].Workspace_model_is_set_<<endl;
			cout<<"Robots_[i].the_LPV_is_set_ "<<Robots_[i].the_LPV_is_set_<<endl;
			cout<<"Robots_[i].the_desired_state_is_set_ "<<Robots_[i].the_desired_state_is_set_<<endl;
		}
	}
	if (Object_.Object_state_is_set_==false)
	{
		cout<<"Object_state_is_set_ is missing"<<endl;
		flag=false;
	}
	for (int i=0;i<Object_.N_grabbing_pos_;i++)
	{
		if ((Object_.Grabbing_state_is_set_[i]==false)||(Vobject_.Grabbing_state_is_set[i]==false))
		{
			cout<<"Sth from grabbing state is missing"<<endl;
			cout<<"Object_.Grabbing_state_is_set_ "<<Object_.Grabbing_state_is_set_[i]<<endl;
			cout<<"Vobject_.Grabbing_state_is_set "<<Vobject_.Grabbing_state_is_set[i]<<endl;
			flag=false;
		}
	}

	return flag;
}
