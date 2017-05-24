#include "Object.h"

const double MIN_SPEED = 0.05;
const double MAX_SPEED = 0.8;


Object::Object()
{

}


void Object::compute_N_DX_O(VectorXd Sum_DX_O)
{
    N_DX_O = DX_O;
    for(int i = 0; i < N_DX_O.rows(), i < Sum_DX_O.rows(); i++)
    {
        N_DX_O(i) = DX_O(i)/Sum_DX_O(i);
    }
 //   cout << "done computing normalized velocity of object " << id << " " << N_DX_O.transpose() << endl;
}


VectorXd Object::get_N_DX_O() const
{
    return N_DX_O;
}

Object::Object(int N_state_, VectorXd X_, VectorXd DX_, double max_time_, double dt_, VectorXd grabbing_states_[], int n_grabbing_states_, double weight_, double value_, int id_, Object_prediction_type Object_motion )
{
	n_state = N_state_;
	id = id_;

	X_O.resizeLike(X_); X_O.setZero(); X_O = X_;
	DX_O.resizeLike(DX_); DX_O.setZero(); DX_O = DX_;


	max_pred_time = max_time_;
	dt = dt_;
	n_frames = floor(max_pred_time/dt)+1;
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


	motion_type = Object_motion;

	double gravity[3];gravity[0]=0;gravity[1]=0;

	if (motion_type==Object_prediction_type::Ballistic)
	{
		gravity[2]=9.81;
	}
	else
	{
		gravity[2]=0;
	}

    status = Object_status::Unallocated;
}

void Object::set_assigned()
{
    status = Object_status::Allocated;
}

void Object::set_done()
{
    status = Object_status::Done;
}

bool Object::is_done() const
{
    return (status == Object_status::Done);

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

void Object::set_status(Object_status stat)
{
    status = stat;
}

Object_status Object::get_status() const
{
    return status;
}

double Object::get_travel_time(int i) const
{
    if(i >= 0 && i < N_CHECKPOINTS)
        return travel_time[i];
    return 0;
}

double Object::get_N_travel_time(int i) const
{
    if(i >= 0 && i < N_CHECKPOINTS)
        return N_travel_time[i];
    return 0;
}

void Object::compute_travel_time(double avg_times[])
{
    for(int i = 0; i < N_CHECKPOINTS; i++)
    {
        if(DX_O(0) > 0 && X_O(0) < X_checkpoint[i])
        {
            travel_time[i]  = (X_checkpoint[i] - X_O(0))/DX_O(0);
        }
        else
            travel_time[i] = -1;

        if(avg_times[i] != 0)
            N_travel_time[i] = travel_time[i]/avg_times[i];
        else
            N_travel_time[i] = -1;
    }
//    cout << "object " << id << " N travel " << N_travel_time[0] << " travel time " << travel_time[0] << " for avg " << avg_times[0] << " DX_O " << DX_O(0) << endl;
}


void Object::dumb_predict_motion()
{
    if(DX_O(0) > 0.1)
    {
        double dist = OBJECT_MAX_X - X_O(0);
        int frames = dist/(DX_O(0)*dt);
        frames = min(frames, n_frames);
        P_O_prediction.resize(X_O.size(), frames+1); // pat hack. Need to find a better way.
        P_O_prediction.setZero();

        //	cout << "P_O_prediction " << endl << P_O_prediction << endl;
        //	cout << "X_O " << endl << X_O << endl;
        //	cout << "DX_O " << endl << DX_O << endl;

        for(int i = 0; i <= frames; i++)
        {
            P_O_prediction.col(i) = X_O + i*dt*DX_O;
        }

        //	cout << "P_O predicted " << endl << P_O_prediction << endl;

        for(int j = 0; j < n_grabbing_pos; j++)
        {
            P_O_G_prediction[j].resize(X_O.size(), frames+1);
            P_O_G_prediction[j].setZero();

    //		cout << "P_O_G prediction " << j << endl << P_O_G_prediction[j] << endl;
    //		cout << "X_O_G " << j << endl << X_O_G[j] << endl;
            for(int k = 0; k <= frames; k++)
            {
                P_O_G_prediction[j].col(k) = P_O_prediction.col(k)+X_O_G[j];//.block(0,0,3,1).transpose();
            }
    //		cout << "for grabbing pos " << j << endl << P_O_G_prediction[j] << endl;
        }

        //	cout << "P_O predicted for " << n_frames << " frames" << endl << P_O_prediction << endl;

    }
    else
    {
        P_O_prediction.resize(X_O.size(), 1);
        P_O_prediction.setZero();

        P_O_prediction.col(0) = X_O;

        for(int j = 0; j < n_grabbing_pos; j++)
        {
            P_O_G_prediction[j].resize(X_O.size(), 1);
            P_O_G_prediction[j].setZero();

            P_O_G_prediction[j].col(0) = P_O_prediction.col(0)+X_O_G[j];
        }
    }


    // old functionning way but sub-optimal.
/*	P_O_prediction.resize(X_O.size(), n_frames+1);
	P_O_prediction.setZero();
//	cout << "P_O_prediction " << endl << P_O_prediction << endl;
//	cout << "X_O " << endl << X_O << endl;
//	cout << "DX_O " << endl << DX_O << endl;

	for(int i = 0; i <= n_frames; i++)
	{
		P_O_prediction.col(i) = X_O + i*dt*DX_O;
	}

//	cout << "P_O predicted " << endl << P_O_prediction << endl;

	for(int j = 0; j < n_grabbing_pos; j++)
	{
		P_O_G_prediction[j].resize(X_O.size(), n_frames+1);
		P_O_G_prediction[j].setZero();

//		cout << "P_O_G prediction " << j << endl << P_O_G_prediction[j] << endl;
//		cout << "X_O_G " << j << endl << X_O_G[j] << endl;
		for(int k = 0; k <= n_frames; k++)
		{
			P_O_G_prediction[j].col(k) = P_O_prediction.col(k)+X_O_G[j];//.block(0,0,3,1).transpose();
		}
//		cout << "for grabbing pos " << j << endl << P_O_G_prediction[j] << endl;
	}

//	cout << "P_O predicted for " << n_frames << " frames" << endl << P_O_prediction << endl;
*/
}


void Object::set_state(VectorXd X, VectorXd DX)
{

	if ((X.rows()!=n_state)||(DX.rows()!=n_state))
	{
		cout<<"The dimension of the state of the object is wrong"<<endl;
	}

	X_O = X;
	DX_O = DX;

}

void Object::set_prediction_parameters(double max_t, double dt_)
{
	max_pred_time = max_t;
	dt = dt_;
	n_frames = floor(max_pred_time/dt)+1;
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
bool Object::get_assignment() const
{
    return ((status == Object_status::Allocated) || (status == Object_status::Grabbed));
}


int Object::get_n_grippers() const
{
	return n_grabbing_pos;
}

double Object::update_value()
{
// on one side, use sigmoid, on the other, use negation
    double value_end = 5*X_O(0)+30;
  //  double speed = DX_O.block(0,0,3,1).norm();
    double speed = DX_O(0);
//    double s1 = sigmoid(speed-MIN_SPEED); // purposedly done in Robot_agent to allow to allocate to the objects when they are in the workspace
//    double s2 = sigmoid(MAX_SPEED-speed);
//    cout << "speed: " << speed << " s1 " << s1 << " s2 " << s2 << endl;
//    value = value_end*s1;
    value = value_end;
    value *= 2;
	if(n_grabbing_pos > 1)
	{
		value *= 2.1;
	}
    if(X_O(0) > OBJECT_MAX_X) // too late to catch
	{
		value = 0;
	//	cout << "object " << id << " is set to 0 because X_O(0) is " << X_O(0) << endl;
	}
	// */
	return value;

}
double Object::get_value() const
{
	return value;
}

double Object::get_weight() const
{
	return weight;
}

MatrixXd Object::get_P_O_prediction() const
{
	return P_O_prediction;
}

MatrixXd Object::get_P_O_G_prediction(int index) const
{
	if(index >= 0 && index < n_grabbing_pos)
		return P_O_G_prediction[index];
    ROS_INFO_STREAM("received wrong parameter for POG_prediction" << endl;);
	return P_O_G_prediction[0];
}

VectorXd Object::get_X_O() const
{
	return X_O;
}

VectorXd Object::get_DX_O() const
{
	return DX_O;
}


int Object::get_id() const
{
	return id;
}

void Object::set_position(const geometry_msgs::Pose &  msg)
{
	X_O(0) = msg.position.x;
	X_O(1) = msg.position.y;
	X_O(2) = msg.position.z;
//	cout << "received new position" << endl << X_O << endl;
}

void Object::set_velocity(const geometry_msgs::Pose & msg)
{
	X_O(3) = msg.position.x;
	X_O(4) = msg.position.y;
	X_O(5) = msg.position.z;
	DX_O(0) = X_O(3);
	DX_O(1) = X_O(4);
	DX_O(2) = X_O(5);
}

void Object::set_accel(const geometry_msgs::Pose & msg)
{
	DX_O(3) = msg.position.x;
	DX_O(4) = msg.position.y;
	DX_O(5) = msg.position.z;
}


void Object::print_estimator() const
{
//	cout << predict << endl;
}

std::ostream& operator <<(std::ostream& stream, const Object& o)
{
	cout << "******PRINTING AN OBJECT******" << endl;
	cout << "max_grabbing_state " << o.max_grabbing_state << endl;
	cout << "max pred time " << o.max_pred_time << endl;
	cout << "weight " << o.weight << endl;
	cout << "value " << o.value << endl;
	cout << "dt " << o.dt << endl;
	cout << "object id " << o.id << endl;
    cout << "status " << static_cast<typename std::underlying_type<Object_status>::type>(o.status) << endl;

    for(int i = 0; i < 3; i++)
	{
		cout << "minPos i " << i << " value " << o.minPos[i] << endl;
	}
	for(int i = 0; i < 3; i++)
	{
		cout << "maxPos i " << i << " value " << o.maxPos[i] << endl;
	}

	cout << "n_state " << o.n_state << endl;



	cout << "max pred time " << o.max_pred_time << endl;

	cout << "n grabbing pos " << o.n_grabbing_pos << endl;

	printVector("X_O_First", o.X_O_First);
	printVector("X_O", o.X_O);

	printVector("DX_O", o.DX_O);



    return cout << "******DONE PRINTING AN OBJECT******" << endl;
}


