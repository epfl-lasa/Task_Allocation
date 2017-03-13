#include "Object.h"


Object::Object()
{
}

Object::Object(int N_state_, VectorXd X_, VectorXd DX_, double max_time_, VectorXd grabbing_states_[], int n_grabbing_states_, double weight_, double value_, ENUM_State_of_prediction Object_motion )
{
	n_state = N_state_;
	X_O = X_;
	DX_O = DX_;
	max_pred_time = max_time_;
	n_grabbing_pos = n_grabbing_states_;
	for(int i = 0; i < n_grabbing_pos; i++)
	{
		X_O_G[i] = grabbing_states_[i];
	}

	weight = weight_;
	value = value_;

	double gravity[3];gravity[0]=0;gravity[1]=0;

	motion_type = Object_motion;
	if (motion_type == Ballistic)
	{
		gravity[2]=9.81;
	}
	else
	{
		gravity[2]=0;
	}

	predict = new TrajectoryEstimation(gravity, dt,30,10);

	state_is_set = true;
	first_state_is_set = true;

	index_column = 0;
	index_row = 0;
	max_liklihood = 0;
	n_frames = -1;
}

Object::~Object()
{
	if(predict != NULL)
		delete predict;
}

void Object::predict_motion()
{
	if ((predict->mReadyToPredict)&&(X_O(0)<4*minPos[0]))
		{

			n_frames =0;

			// predict the position of center of gravity
			n_frames = predict->PredictNextPosVel(max_pred_time,dt,P_O_prediction,true,0,maxPos[0]);

			if 	(n_frames == -1)
			{
				// no prediction for some reason
				cout << "no prediction available" << endl;
			}
			else
			{
				// predict the positions of each grabbing position
				for (int j = 0; j < n_grabbing_pos; j++)
				{
					for (int k = 0; k < n_frames; k++)
					{
						P_O_G_prediction[j].row(k)=P_O_prediction.row(k)+X_O_G[j].block(0,0,3,1).transpose();
					}
					//	cout<<"Object_.P_O_G_prediction_[i] "<<i<<" "<<Object_.P_O_G_prediction_[i]<<endl;
				}
			}
		}
}

void Object::set_max_pred_time(double time)
{
	if(time > 0)
		max_pred_time = time;
	else
	{
		cout << "error, max pred time is negative : " << time << endl;
	}
}

void Object::set_state(VectorXd X, VectorXd DX)
{

	if (state_is_set==true)
	{
		cout<<"States of the object is already set."<<endl;
	//	ERROR();
	}

	if ((X.rows()!=n_state)&&(DX.rows()!=n_state))
	{
		cout<<"The dimension of the state of the object is wrong"<<endl;
	//	ERROR();
	}

	if(first_state_is_set == false)
	{
		X_O_First = X;
		first_state_is_set = true;
	}
	X_O = X;
	DX_O = DX;

	state_is_set = true;
}


// getters
bool Object::get_state_set()
{
	return state_is_set;
}


bool Object::get_first_state_set()
{
	return first_state_is_set;
}

// defaults to false if index out of range
bool Object::get_grabbing_state_set(int i)
{
	if(i >= 0 && i < n_grabbing_pos)
		return grabbing_state_is_set[i];

	return false;
}

double Object::get_value()
{
	return value;
}

double Object::get_weight()
{
	return weight;
}
