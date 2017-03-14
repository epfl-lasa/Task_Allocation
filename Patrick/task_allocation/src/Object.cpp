#include "Object.h"


Object::Object(int n_state_)
{
	n_state = n_state_;
	cout << "I'm making an Object" << endl;

	//cout << Ballistic_patrick << " " << Straight_patrick << endl;
}

Object::Object(int N_state_, VectorXd X_, VectorXd DX_, double max_time_, VectorXd grabbing_states_[], int n_grabbing_states_, double weight_, double value_, Object_prediction_type Object_motion )
{

	cout << "I'm making an Object" << endl;
	n_state = N_state_;


	X_O.resizeLike(X_); X_O.setZero(); X_O = X_;
	DX_O.resizeLike(DX_); DX_O.setZero(); DX_O = DX_;



	max_pred_time = max_time_;
	n_grabbing_pos = n_grabbing_states_;
	for(int i = 0; i < n_grabbing_pos; i++)
	{
		X_O_G[i].resizeLike(grabbing_states_[i]); X_O_G[i].setZero(); X_O_G[i] = grabbing_states_[i];
	}

	weight = weight_;
	value = value_;

	P_O_prediction.resize((int)floor(max_pred_time/(dt))+1,3); P_O_prediction.setZero();
	for(int i = 0; i < n_grabbing_pos; i++)
	{
		P_O_G_prediction[i].resize((int)floor(max_pred_time/(dt))+1,3); P_O_G_prediction[i].setZero();

	}

	double gravity[3];gravity[0]=0;gravity[1]=0;

	motion_type = Object_motion;
	if (motion_type == Object_prediction_type::Ballistic)
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
	else
	{
		cout << "cant predict" << endl;
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


void Object::set_prediction_state(VectorXd X,VectorXd X_filtered, double time)
{
	if ((X_filtered.rows()!=3))
		{
			cout<<"The dimension of the state of the object is wrong"<<endl;
			//ERROR();
		}


		X_O.block(0,0,3,1)=X;
		if (first_state_is_set==false)
		{
			first_state_is_set=true;
			X_O_First=X_O.block(0,0,3,1);
			cout<<"The first position of the object is saved:"<<endl;
			cout<<X_O_First<<endl;
		}
	//	else if ((first_state_is_set==true)&&(fabs(X_O(0)-X_O_First(0))>0.4))
		else
		{
			predict->setNewObservationset(time, X_filtered);
		}
		state_is_set=true;
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

MatrixXd Object::get_P_O_prediction()
{
	return P_O_prediction;
}

MatrixXd Object::get_P_O_G_prediction(int index)
{
	if(index > 0 && index < n_grabbing_pos)
		return P_O_G_prediction[index];
	return P_O_G_prediction[0];
}

VectorXd Object::get_X_O()
{
	return X_O;
}

VectorXd Object::get_DX_O()
{
	return DX_O;
}
