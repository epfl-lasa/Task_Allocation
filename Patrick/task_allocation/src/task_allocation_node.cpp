#include "task_allocation_node.h"

const int N_ROBOTS = 4;
const int N_OBJECTS = 4;

const double dt = 0.030; // seconds
const double max_time = 2; // seconds
const int n_state = 6;
int countpp = 0;


/*
std::vector<Vector3d> rob_bases;
std::vector<Vector3d> rob_ends;
std::vector<VectorXd> obj_state;
std::vector<VectorXd> dobj_state;
*/

int start = 42;
ros::Subscriber start_sub;

std_msgs::Int64 rob_id_msg;

ros::NodeHandle *n;

std::vector<ros::Publisher> rob_target_pub;
std::vector<ros::Publisher> rob_target_id_pub;
std::vector<ros::Publisher> rob_coordination_pub;

std::vector<ros::Publisher> rob_grabbed_pub;

std::vector<ros::Subscriber> rob_end_sub;
std::vector<ros::Subscriber> rob_base_sub;

std::vector<ros::Subscriber> obj_pos_sub;
std::vector<ros::Subscriber> obj_vel_sub;
std::vector<ros::Subscriber> obj_acc_sub;
std::vector<ros::Publisher> obj_done_pub;




std::vector<Object> Objects;
std::vector<Robot_agent> Robots;


Task_allocation* Task_allocator;

void chatter_ready(const std_msgs::Int64 msg)
{
	start = msg.data;
}


int main(int argc, char **argv) {

	MatrixXd targets;
	std::vector<double> coordinations;

	ros::init(argc, argv, "task_allocation");

	n = new ros::NodeHandle();

	init();

	prepare_task_allocator();

	init_topics();


	cout << "done initializing task allocation" << endl;

	cout << "allocating " << N_ROBOTS << " robots to " << N_OBJECTS << " objects" << endl;
	cout << "waiting for simulator start" << endl;
	// while we haven't received everything, wait.. This is not very orthodox
	while(start != 0) // 0 is the value we get from the "init" in the "job init catch" sequence
	{
		if(ros::ok())
		{
			ros::spinOnce();
		}
	}

    for(auto & rob : Robots)
    {

        cout << rob << endl;
    }
	cout << "starting allocating in a loop" << endl;

	// loop...
	ros::Rate r(5);
	while(ros::ok())
	{

		Task_allocator->update_objects_value();
		Task_allocator->predict_motion();
		Task_allocator->update_rob_business();
		Task_allocator->allocate();
		Task_allocator->compute_intercepts();
		Task_allocator->compute_coordination();


		// check at what state the robots are
		for(auto & rob : Robots)
		{
			int assign = rob.get_assignment();
			if(assign != -1) // if it's assigned
			{
				if(Objects[assign].get_n_grippers() == 1) // if its object is a "1 robot" object
				{
					double distance = (rob.get_end() - Objects[assign].get_X_O().block(0,0,3,1)).norm();
	//				cout << "robot " << rob.get_id() << " is at " << distance << " of its target" << endl;
					if( distance < 0.3) // if it reached the object
					{
						cout << "robot " << rob.get_id() << " grabbed object " << Objects[assign].get_id() << endl;
						rob.set_grabbed(true);

					}
				}
			}


			if(rob.get_grabbed()) // if the robot has grabbed something
			{
				rob.set_idle(); // keep sending it to idle position
	//			rob.update_business();
		//		if(!(rob.is_busy())) // if it reached its idle position, make it drop the object, mark object as done.
				if((rob.get_idle_pos() - rob.get_end()).norm() < 0.3)
				{
					Objects[rob.get_assignment()].set_done();
					rob.set_grabbed(false);
					cout << "robot " << rob.get_id() << " released the object " << Objects[rob.get_assignment()].get_id() << endl;
				}
			}
		}

//



		// publish stuff
		targets = Task_allocator->get_targets();
		coordinations = Task_allocator->get_coordinations();

		// publish who grabbed what.
		for(auto & rob : Robots)
		{
			int assign = rob.get_assignment();
			std_msgs::Int64 msg;
			msg.data = -1;
			if(assign >= 0)
			{
				if(Objects[assign].get_n_grippers() == 1)
				{
					if(rob.get_grabbed())
						msg.data = assign;

				}
			}

			rob_grabbed_pub[rob.get_id()].publish(msg);
		}


		// publish the desired coordinations...
		for(int i = 0; i < coordinations.size(); i++)
		{
			std_msgs::Float64 msg;
			msg.data = coordinations[i];
			rob_coordination_pub[i].publish(msg);
		}

		// publish the targets...
		for(int i = 0; i < targets.cols(); i++)
		{
			geometry_msgs::Pose msg;
			msg.position.x = (targets.col(i))(0);
			msg.position.y = (targets.col(i))(1);
			msg.position.z = (targets.col(i))(2);
			msg.orientation.w = 1;
			rob_target_pub[i].publish(msg);
		}

		// publish the ids for the coalitions for display
		for(int i = 0; i < N_ROBOTS; i++)
		{
			rob_id_msg.data = Task_allocator->get_robot_target(i);
			rob_target_id_pub[i].publish(rob_id_msg);
		}


		for(int i = 0; i < N_OBJECTS; i++)
		{
			std_msgs::Int64 msg;
			msg.data = Objects[i].is_done();
			obj_done_pub[i].publish(msg);

		}





//		cout << *Task_allocator << endl;
		ros::spinOnce();
		r.sleep();
	}
}

void init()
{

}


void init_topics()
{
	std::ostringstream oss;

	start_sub = n->subscribe("/command", 1, chatter_ready);
	// robots
	for(int i = 0; i < Robots.size(); i++)
	{
		// publishers
		oss.str("");
		oss.clear();
		oss << "/robotsPat/targetID/" << i;
		rob_target_id_pub.push_back(n->advertise<std_msgs::Int64>(oss.str(), 1));

		oss.str("");
		oss.clear();
		oss << "/robotsPat/grabbed/" << i;
		rob_grabbed_pub.push_back(n->advertise<std_msgs::Int64>(oss.str(), 1));


		oss.str("");
		oss.clear();
		oss << "/robotsPat/coordination/" << i;
		rob_coordination_pub.push_back(n->advertise<std_msgs::Float64>(oss.str(), 1));

		oss.str("");
		oss.clear();
		oss << "/robotsPat/target/" << i;
		rob_target_pub.push_back(n->advertise<geometry_msgs::Pose>(oss.str(), 1));

		// subscribers
		oss.str("");
		oss.clear();
		oss << "/robot/base/" << i;
		rob_base_sub.push_back(n->subscribe(oss.str(), 1, &Robot_agent::set_base, &(Robots[i])));

		oss.str("");
		oss.clear();
		oss << "/robot_real/end/" << i;
		rob_end_sub.push_back(n->subscribe(oss.str(), 1, &Robot_agent::set_end, &(Robots[i])));


	}


	// objects
	for(int i = 0; i < Objects.size(); i++)
	{
		oss.str("");
		oss.clear();
		oss << "/object/p" << i << "/position";
		obj_pos_sub.push_back(n->subscribe(oss.str(), 1, &Object::set_position, &(Objects[i])));

		oss.str("");
		oss.clear();
		oss << "/object/p" << i << "/velocity";
		obj_vel_sub.push_back(n->subscribe(oss.str(), 1, &Object::set_velocity, &(Objects[i])));

		oss.str("");
		oss.clear();
		oss << "/object/p" << i << "/acceleration";
		obj_acc_sub.push_back(n->subscribe(oss.str(), 1, &Object::set_accel, &(Objects[i])));

		oss.str("");
		oss.clear();
		oss << "/object/p" << i << "/done";
		obj_done_pub.push_back(n->advertise<std_msgs::Int64>(oss.str(), 1));
	}
}


void prepare_task_allocator()
{
	double dt = 0.030; // seconds
	double max_time = 2; // seconds
	int n_state = 6;

	Task_allocator = new Task_allocation(max_time, dt, n_state);

	// adding object(s)
	add_objects_task_allocator();

	//adding robot(s)
	add_robots_task_allocator();

}


void add_objects_task_allocator()
{
	cout << "adding objects" << endl;
//	VectorXd X_O_G[N_grabbing];
	VectorXd zeroVec;
	zeroVec.resize(6); zeroVec.setZero();
/*	for(int i = 0; i < N_grabbing; i++)
	{
		X_O_G[i] = Object_Grabbing_State[i] - Object_State_raw;
	}
*/

	double weight = 3.14;
	double value = 15;
	VectorXd single_grab[1];
	single_grab[0].resize(n_state); single_grab[0].setZero();

	VectorXd double_grab[2];
	double_grab[0] = single_grab[0];
	double_grab[1] = single_grab[0];
	Object task0(n_state,zeroVec,zeroVec, Task_allocator->get_max_time(), Task_allocator->get_dt(), double_grab, 2, weight, value, 0);
	Object task1(n_state,zeroVec,zeroVec, Task_allocator->get_max_time(), Task_allocator->get_dt(), single_grab, 1, weight*0.3, value, 1);
	Object task2(n_state,zeroVec,zeroVec, Task_allocator->get_max_time(), Task_allocator->get_dt(), single_grab, 1, weight*0.3, value, 2);
	Object task3(n_state,zeroVec,zeroVec, Task_allocator->get_max_time(), Task_allocator->get_dt(), single_grab, 1, weight*0.3, value, 3);


	Objects.push_back(task0);
	Objects.push_back(task1);
	Objects.push_back(task2);
	Objects.push_back(task3);


	for(auto & obj : Objects)
		Task_allocator->add_task(&obj);
}


void add_robots_task_allocator()
{

	cout << "adding robots" << endl;
	double force = 5;
	int grippers = 1;

	for(int i = 0; i < N_ROBOTS; i++)
	{

		Vector3d base;
		base.setZero();
		Robot_agent Bot(1,addTwochar(Common_path,"/A_Matrix").c_str(), addTwochar(Common_path,"/Priors").c_str(),
						addTwochar(Common_path,"/Mu").c_str(), addTwochar(Common_path,"/Sigma").c_str(),
						6, 3, addTwochar(Common_path,"/IIWA_workspace_Model_prior").c_str(),
						addTwochar(Common_path,"/IIWA_workspace_Model_mu").c_str(),
						addTwochar(Common_path,"/IIWA_workspace_Model_Sigma").c_str(),
						addTwochar(Common_path,"/IIWA_workspace_Model_Threshold").c_str(), base, i, grippers, force);

		Robots.push_back(Bot);

	//	cout << "robot " << i << " base " << base << endl;
	}

	for(auto & rob : Robots)
		Task_allocator->add_robot(&rob);
}

