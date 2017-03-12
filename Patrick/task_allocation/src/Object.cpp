#include "Object.h"


Object::Object()
{
}

Object::Object(int N_state_, VectorXd X_, VectorXd DX_, double max_time_, VectorXd grabbing_states_[], int n_grabbing_states_, double weight_, double value_ )
{
	N_state = N_state_;
	X_O = X_;
	DX_O = DX_;
	max_pred_time = max_time_;
	N_grabbing_pos = n_grabbing_states_;
	for(int i = 0; i < N_grabbing_pos; i++)
	{
		X_O_G_[i] = grabbing_states_[i];
	}

	weight = weight_;
	value = value_;

}

void Object::predict_motion()
{
	if ((predict->mReadyToPredict)&&(X_O(0)<4*minPos[0]))
		{

			N_frames =0;

			// predict the position of center of gravity
			N_frames = predic_->PredictNextPosVel(max_pred_time,dt,P_O_prediction,true,0,maxPos[0]);

			if 	(N_frames==-1)
			{
				// no prediction for some reason
				cout << "no prediction available" << endl;
			}
			else
			{
				// predict the positions of each grabbing position
				for (int j=0;j<N_grabbing_pos;j++)
				{
					for (int k=0;k<N_frames;k++)
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
	if (State_is_set==true)
	{
		cout<<"States of the object is already set."<<endl;
	//	ERROR();
	}

	if ((X.rows()!=N_state)&&(DX.rows()!=N_state))
	{
		cout<<"The dimension of the state of the object is wrong"<<endl;
	//	ERROR();
	}

	X_O = X;
	DX_O = DX;

	State_is_set = true;
}



// getter
bool Object::get_state_set()
{
	return State_is_set;
}


bool Object::get_first_state_set()
{
	return First_state_is_set;
}


bool Object::get_grabbing_state_set(int i)
{
	return Grabbing_state_is_set[i];
}


double Object::get_value()
{
	return value;
}

double Object::get_weight()
{
	return weight;
}
