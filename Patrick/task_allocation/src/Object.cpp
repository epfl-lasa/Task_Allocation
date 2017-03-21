#include "Object.h"

Object::Object()
{

}

Object::Object( const Object &o) // copy construct
{
	//cout << "something called the copy constructor" << endl;

	n_state = o.n_state;
	//state_is_set = o.state_is_set;
	first_state_is_set = o.first_state_is_set;

	for(int i = 0; i < max_grabbing_state; i++)
	{
	//	grabbing_state_is_set[i] = o.grabbing_state_is_set[i];
		X_O_G[i] = o.X_O_G[i];
	}

	motion_type = o.motion_type;
	max_pred_time = o.max_pred_time;
	n_grabbing_pos = o.n_grabbing_pos;
	X_O_First = o.X_O_First;
	X_O = o.X_O;
//	X_O_INTERCEPT = o.X_O_INTERCEPT;
//	X_I_C = o.X_I_C;
	DX_O = o.DX_O;

	is_assigned = o.is_assigned;
/*	double gravity[3];gravity[0]=0;gravity[1]=0;
	if (motion_type == Object_prediction_type::Ballistic)
	{
		gravity[2]=9.81;
	}
	else
	{
		gravity[2]=0;
	}
//	predict = new TrajectoryEstimation(gravity, dt,30,10); // DANGEROUS HERE.
*/
	n_frames = o.n_frames;
	P_O_prediction = o.P_O_prediction;
	for(int i = 0; i < max_grabbing_state; i++)
	{
		P_O_G_prediction[i] = o.P_O_G_prediction[i];
	}

//	order_of_grabbing = o.order_of_grabbing;
//	prob_order_of_grabbing = o.prob_order_of_grabbing;

	weight = o.weight;
	value = o.value;

//	max_liklihood = 0;
//	index_row = 0;
//	index_column = 0;

//	cout << o << endl;
//	cout << *this << endl;

}
Object::Object(int N_state_, VectorXd X_, VectorXd DX_, double max_time_, VectorXd grabbing_states_[], int n_grabbing_states_, double weight_, double value_, int id_, Object_prediction_type Object_motion )
{
//	cout << "I'm making an Object" << endl;
	n_state = N_state_;
	id = id_;

	X_O.resizeLike(X_); X_O.setZero(); X_O = X_;
	DX_O.resizeLike(DX_); DX_O.setZero(); DX_O = DX_;


	max_pred_time = max_time_;
	if(n_grabbing_states_ > max_grabbing_state)
		cout << "error, n grabbing bigger than max grabbing" << endl;

	n_grabbing_pos = n_grabbing_states_;
	for(int i = 0; i < n_grabbing_pos; i++)
	{
		X_O_G[i] = grabbing_states_[i];
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

	first_state_is_set = true;

	is_assigned = false;
	n_frames = -1;

}

void Object::set_assigned()
{
	is_assigned = true;
}

void Object::predict_motion()
{
/*	if ((predict->mReadyToPredict)&&(X_O(0)<4*minPos[0]))
	{
		n_frames =0;

		// predict the position of center of gravity
		n_frames = predict->PredictNextPosVel(max_pred_time,dt,P_O_prediction,true,0,maxPos[0]);

		if 	(n_frames == -1)
		{

			// no prediction for some reason
		//	cout << "no prediction available" << endl;
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

*/
}

void Object::dumb_predict_motion()
{
	int pred_frame = 3;
	P_O_prediction.resize(X_O.size(), pred_frame+1);
	P_O_prediction.setZero();
//	cout << "P_O_prediction " << endl << P_O_prediction << endl;
//	cout << "X_O " << endl << X_O << endl;
//	cout << "DX_O " << endl << DX_O << endl;

	for(int i = 0; i <= pred_frame; i++)
	{
		P_O_prediction.col(i) = X_O + i*dt*DX_O;
	}

//	cout << "P_O predicted " << endl << P_O_prediction << endl;

	for(int j = 0; j < n_grabbing_pos; j++)
	{
		P_O_G_prediction[j].resize(X_O.size(), pred_frame+1);
		P_O_G_prediction[j].setZero();

//		cout << "P_O_G prediction " << j << endl << P_O_G_prediction[j] << endl;
//		cout << "X_O_G " << j << endl << X_O_G[j] << endl;
		for(int k = 0; k <= pred_frame; k++)
		{
			P_O_G_prediction[j].col(k) = P_O_prediction.col(k)+X_O_G[j];//.block(0,0,3,1).transpose();
		}
//		cout << "for grabbing pos " << j << endl << P_O_G_prediction[j] << endl;
	}

//	cout << "P_O predicted " << endl << P_O_prediction << endl;
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

/*	if (state_is_set==true)
	{
		cout<<"States of the object is already set."<<endl;
	//	ERROR();
	}
*/
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

//	state_is_set = true;
}


void Object::set_prediction_state(VectorXd X,VectorXd X_filtered, double time)
{
/*	if ((X_filtered.rows()!=3))
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

		*/
}

// getters


bool Object::get_first_state_set()
{
	return first_state_is_set;
}

int Object::get_n_grippers()
{
	return n_grabbing_pos;
}

// defaults to false if index out of range
bool Object::get_grabbing_state_set(int i)
{
//	if(i >= 0 && i < n_grabbing_pos)
//		return grabbing_state_is_set[i];
	return true;
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



std::ostream& operator <<(std::ostream& stream, const Object& o)
{
	cout << "******PRINTING AN OBJECT******" << endl;
	cout << "max_grabbing_state " << o.max_grabbing_state << endl;
	cout << "dt " << o.dt << endl;
	for(int i = 0; i < 3; i++)
	{
		cout << "minPos i " << i << " value " << o.minPos[i] << endl;
	}
	for(int i = 0; i < 3; i++)
	{
		cout << "maxPos i " << i << " value " << o.maxPos[i] << endl;
	}

	cout << "n_state " << o.n_state << endl;

//	cout << "state is set " << o.state_is_set << endl;
	cout << "first state is set " << o.first_state_is_set << endl;

//	for(int i = 0; i < o.max_grabbing_state; i++)
//		cout << "grabbing_state_is_set i = " << i << " value  " << o.grabbing_state_is_set[i] << endl;

	cout << "max pred time " << o.max_pred_time << endl;

	cout << "n grabbing pos " << o.n_grabbing_pos << endl;

	printVector("X_O_First", o.X_O_First);
	printVector("X_O", o.X_O);
//	printVector("X_O_INTERCEPT", o.X_O_INTERCEPT);
//	printVector("X_I_C", o.X_I_C);
	printVector("DX_O", o.DX_O);

	cout << "weight " << o.weight << endl;
	cout << "value " << o.value << endl;

	cout << "******DONE PRINTING AN OBJECT******" << endl;
}


