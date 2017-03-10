#include "Object.h"

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
	if (State_is_set_==true)
	{
		cout<<"States of the object is already set."<<endl;
	//	ERROR();
	}

	if ((X.rows()!=N_state_)&&(DX.rows()!=N_state_))
	{
		cout<<"The dimension of the state of the object is wrong"<<endl;
	//	ERROR();
	}

	X_O_=X;
	DX_O_=DX;

	State_is_set_=true;
}



// getter
bool Object::get_state_set()
{
	return State_is_set_;
}


bool Object::get_first_state_set()
{
	return First_state_is_set_;
}


bool Object::get_grabbing_state_set(int i)
{
	return Grabbing_state_is_set_[i];
}


double Object::get_value()
{
	return value;
}

double Object::get_weight()
{
	return weight;
}
