/*
 * Robot_agent.cpp
 *
 *  Created on: 12 mars 2017
 *      Author: Patrick
 */

#include "Robot_agent.h"


Robot_agent::Robot_agent()
{

}

Robot_agent::Robot_agent(int Num_LPV_Com, const char *path_A_LPV, const char *path_prior_LPV,const char *path_mu_LPV, const char *path_sigma_LPV,
				int Num_GMM_Com, int Num_GMM_state, const char *path_prior_GMM, const char *path_mu_GMM, const char *path_sigma_GMM,
				const char *path_threshold, Vector3d X_Base, int ID, int grip, double force_)
{


	int n_state = 6;

	cout << "begin creating a robot " << endl;
	Workspace_model.initialize(Num_GMM_Com,Num_GMM_state);
    //Dynamic.initialize(Num_LPV_Com,n_state);

	Workspace_model.initialize_GMM(path_prior_GMM,path_mu_GMM,path_sigma_GMM,path_threshold);
    //Dynamic.initialize_theta(path_prior_LPV,path_mu_LPV,path_sigma_LPV);
    //Dynamic.initialize_A(path_A_LPV);
	X_base = X_Base;

	X.resize(n_state); X.setZero();

    //tau = 0.0001;
    //Dtau = 0;

	id = ID;
	n_grippers = grip;
	force = force_;

	assignment = -1;

	busy = 0;
	X_idle = X_base + Vector3d(-0.5,0,0.6);
	set_idle();

	cout << "done" << endl;
}


void Robot_agent::set_assignment(int assignment_)
{
	assignment = assignment_;
   /* if(id == 0)
    {
        cout << "robot 0 setting status to " << assignment;
    }*/
    if(assignment < 0)
    {
       // cout << " and is unallocated" << endl;
        status = Robot_status::Unallocated;
    }
    else
    {
        //cout << " and is allocated" << endl;
        status = Robot_status::Allocated;
    }
}

int Robot_agent::get_assignment() const
{
	return assignment;
}


void Robot_agent::get_state(VectorXd& X_) const
{
	X_ = X;
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
	}

	X = X_;
}

Robot_status Robot_agent::get_status() const
{
    return status;
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

Vector3d Robot_agent::get_end() const
{
	return X_end;
}
/*
double Robot_agent::sigmoid(double x) const
{
    return 1.0f/(1+exp(-SIGMOID_SLOPE_FACTOR*x));
}
*/

double Robot_agent::evaluate_task(const Object& obj)
{
	int n_grips = obj.get_n_grippers();
	double delta_norm = 1000000;
	double temp_delta;

    double cost = ROBOT_MAX_COST;
    MatrixXd POG[n_grips];

    double temp_prob[n_grips];
    double best_prob[n_grips];
    Vector3d best_pos[n_grips];
    double best_prob_overall = 0;
    double travel_time = 0;
    double rob_travel_dist = 0;
    double rob_travel_time = 0;
    double n_travel_time = 0;
    double obj_speed = obj.get_DX_O()(0); // get velocity
    double n_obj_speed = obj.get_N_DX_O()(0); // get its normalization factor

    double s1 = sigmoid(n_obj_speed-OBJ_MIN_SPEED);
    double s2 = sigmoid(OBJ_MAX_SPEED-obj_speed);



    // probability of being in workspace in future
    for(int i = 0; i < n_grips; i++)
	{
        best_prob[i] = 0;
		POG[i] = obj.get_P_O_G_prediction(i);
  //    cout << "robot " << id << " object " << obj.get_id() << " POG size " << POG[i].rows() << " " << POG[i].cols() << endl;
        for(int j = 0; j < POG[i].cols(); j++)
        {
            if(abs((POG[i].col(j)(0)-X_base(0))) < 1.5)
            {
                VectorXd col = POG[i].col(j).block(0,0,3,1);
           //     cout << col << " has prob " << prob << endl;
              //  cout << col.transpose() << " has prob " << Workspace_model.PDF(col - X_base) << endl;
              //  cout << (POG[i].col(j).block(0,0,3,1) - X_base).transpose() << " has prob " << Workspace_model.PDF((POG[i].col(j).block(0,0,3,1) - X_base)) << endl;
              //  (POG[i].col(j).block(0,0,3,1) - X_base).transpose();
              //  Workspace_model.PDF((POG[i].col(j).block(0,0,3,1) - X_base));


                temp_prob[i] = Workspace_model.PDF((col - X_base));
                if(temp_prob[i] > best_prob[i])
                {
                    best_prob[i] = temp_prob[i];
                    best_pos[i] = col;
                    if(best_prob[i] > 0.25) // pat hack value.
                        break;
                }
           //     cout << "temp prob " << i << " " << temp_prob[i] << endl;
            }
        }
    }

    for(int i = 0; i < n_grips; i++)
    {
        if(best_prob[i] > best_prob_overall && best_prob[i] > 0)
        {
            best_prob_overall = best_prob[i];
            X_targ = best_pos[i];

        }
    }

    if(best_prob_overall <= 0) // impossible, put a high number.
    {
        X_targ = X_end;
        cost = ROBOT_MAX_COST;
    }
    else
    {
        travel_time = obj.get_travel_time(id/ 2); //((X_targ - obj.get_X_O().block(0,0,3,1)).norm())/(obj_speed*n_obj_speed);
        n_travel_time = obj.get_N_travel_time(id/2);


        rob_travel_dist = (X_targ - X_end).norm();
        rob_travel_time = rob_travel_dist/1; // arbitrary max velocity of the robot at 1m/s.


       // cost = n_travel_time*rob_travel_dist/best_prob_overall;

        cost = (travel_time + rob_travel_dist)/best_prob_overall;

        // apply sigmoid from upper velocity boundary
        if(!std::isnan(s2) && s2 != 0)
            cost /= s2;
        else
            cost = ROBOT_MAX_COST;


   //     cout << "robot " << id << " object " << obj.get_id() <<  " dist " << rob_travel_dist << " travel_time " << travel_time << " n_travel " << n_travel_time << " cost " << cost << endl;

        if(rob_travel_dist > 0.25) // only apply the time of travel if the robot will have to travel...
        {
            double s3 = sigmoid(travel_time - rob_travel_time);
            if(!std::isnan(s3) && s3 != 0)
                cost /= s3;
            else
                cost = ROBOT_MAX_COST;
        }

/*        // **************** BACKUP BELOW **************************
        // distance of object
        temp_delta = (X_end.block(0,0,3,1) - obj.get_X_O().block(0,0,3,1)).norm();
        delta_norm = temp_delta;

        cost = delta_norm / best_prob_overall;

        // apply sigmoid from upper boundary
        if(!std::isnan(s2) && s2 != 0)
            cost /= s2;
        else
            cost = ROBOT_MAX_COST;

        // ********************* BACKUP ABOVE ************************/
    //    cout << "robot " << id << " object " << obj.get_id() << " s3 " << s3 << " time object " << travel_time << " time robot " << rob_travel_time << endl;
    //   cout << "travel from " << X_end.transpose() << " to " << X_targ.transpose() << endl;

    }

   /* if(id == 1)
    {
        if(obj.get_id() == 2)
        {

            ROS_INFO_STREAM("Robot " << id << " done computing probability for object " << obj.get_id() << " p=" << best_prob_overall << " target " << X_targ.transpose() << "  object is at " << obj.get_X_O().block(0,0,3,1).transpose() << endl);
            ROS_INFO_STREAM("Robot " << id << " base " << X_base.transpose() << " idle " << X_idle.transpose() << " end " << X_end.transpose());
        }
    }
*/

    //	cout << " POG " << i << endl << POG[i] << endl;
	//	cout << " POG " << i  << " has " << POG[i].cols() << " columns" << endl;
	//	cout << "last column is " << endl << POG[i].col(POG[i].cols()-1) << endl;
//		temp_delta = (X_base.block(0,0,3,1) - (POG[i].col(0)).block(0,0,3,1)).norm();
    //	if(temp_delta < delta_norm)
        //	delta_norm = temp_delta;
//	}

 /*

    // distance of object
    temp_delta = (X_end.block(0,0,3,1) - obj.get_X_O().block(0,0,3,1)).norm();
	delta_norm = temp_delta;



    if(best_prob_overall > 0) // avoid dividing by 0
    {
        cost = (delta_norm + travel_time) / best_prob_overall;

        // apply sigmoid from upper boundary
        if(!std::isnan(s2) && s2 != 0)
            cost /= s2;
        else
            cost = ROBOT_MAX_COST;

    //    cout << "robot " << id << " object " << obj.get_id() << " s1 " << s1 << " s2 " << s2 << " travel time " << travel_time << endl;
    }
    else
    {
        cost = ROBOT_MAX_COST;
    //    cout << "robot " << id << " prob of catch " << best_prob_overall << endl;
    }
*/
    return cost;
}



VectorXd Robot_agent::get_target() const
{
	return X_targ;
}


Vector3d Robot_agent::get_idle_pos() const
{
	return X_idle;
}

void Robot_agent::set_grabbed()
{
    status = Robot_status::Grabbed;
}

void Robot_agent::update_status()
{
//	cout << " end " << endl << X_end << endl;
//	cout << " target " << endl << X_targ << endl;
 //   double delta = (X_end - X_targ.block(0,0,3,1)).norm();

  /*  if(status == Robot_status::Grabbed)
    {
        if((X_idle - X_targ).norm() < 0.01) // is robot going to idle position, unsure if I can just X_idle == X_targ
        {
            if(delta < 0.1)
            {
                status = Robot_status::Unallocated;
                cout << "robot " << id << " went from grabbed to unallocated" << endl;
            }
        }
    }
    */
}


void Robot_agent::set_done()
{
    status = Robot_status::Unallocated;
}

bool Robot_agent::has_grabbed() const
{
    return status == Robot_status::Grabbed;
}


VectorXd Robot_agent::compute_intercept(const Object& obj)
{
	VectorXd best_pos;
	X_targ.resize(3); X_targ.setZero();
	MatrixXd POG = obj.get_P_O_G_prediction(0);//obj.get_P_O_G_prediction(0);
//	cout << "trying to get intercept for object " << obj.get_id() << " POG is of size " << POG.rows() << " " << POG.cols()  << endl;// << POG << endl;
	double best_prob = -1;
	double temp_prob = -1;
	for(int i = 0; i < POG.cols(); i++)
	{
//		cout << "vector " << i << endl << POG.col(i) << endl;
        if(abs((POG.col(i)(0)-X_base(0))) < 1.5)
        {
            temp_prob = Workspace_model.PDF((POG.col(i).block(0,0,3,1) - X_base));
            if(temp_prob > best_prob)
            {
                best_prob = temp_prob;
                best_pos = POG.col(i);//.block(0,0,3,1);
                if(best_prob > 0.2)
                    break;
            }
        }
	}

/*    if(id == 1)
    {
        if(obj.get_id() == 2)
        {

            ROS_INFO_STREAM("Robot " << id << " done computing probability for object " << obj.get_id() << " p=" << best_prob<< " target " << X_targ.transpose() << "  object is at " << obj.get_X_O().block(0,0,3,1).transpose() << endl);
            ROS_INFO_STREAM("Robot " << id << " base " << X_base.transpose() << " idle " << X_idle.transpose() << " end " << X_end.transpose());
        }
    }
*/
	if(best_prob > 0.1)
	{
		X_targ = best_pos.block(0,0,3,1);
	}
	else
	{
        X_targ = X_end;
	}
	return X_targ;
}

void Robot_agent::set_idle()
{
	X_targ.resize(3); X_targ.setZero();
	X_targ = X_idle;
}

void Robot_agent::set_base(const geometry_msgs::Pose & msg)
{
	X_base(0) = msg.position.x;
	X_base(1) = msg.position.y;
	X_base(2) = msg.position.z;

	X_idle = X_base + Vector3d(-0.5,0,0.6);
    set_idle();
	cout << "x base received for robot " << id << endl << X_base << endl << "x_idle is " << endl << X_idle << endl;
}

void Robot_agent::set_end(const geometry_msgs::Pose & msg)
{
	X_end(0) = msg.position.x;
	X_end(1) = msg.position.y;
	X_end(2) = msg.position.z;
}


std::ostream& operator <<(std::ostream& stream, const Robot_agent& o)
{

	cout << "ID " << o.id << endl;
	cout << "X base " << endl << o.X_base << endl;
	cout << "X " << endl << o.X << endl;
	cout << "n grippers " << o.n_grippers << endl;
	cout << "force " << o.force << endl;
    return cout << "assigned to task " << o.assignment << endl;

}
