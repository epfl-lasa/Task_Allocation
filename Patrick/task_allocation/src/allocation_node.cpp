#include "allocation_node.h"

const int N_ROBOTS = 4;
const int N_OBJECTS = 4;

const double dt = 0.030; // seconds
const double max_time = 2; // seconds
const int n_state = 6;

std::vector<Vector3d> bases;
std::vector<Vector3d> ends;
std::vector<VectorXd> obj_state;
std::vector<VectorXd> dobj_state;

ros::NodeHandle n;


std::vector<ros::Publisher> rob_target_id_pub;
std::vector<ros::Publisher> rob_coordination_pub;


std::vector<ros::Subscriber> rob_end_sub;
std::vector<ros::Subscriber> rob_base_sub;

std::vector<ros::Subscriber> obj_pos_sub;
std::vector<ros::Subscriber> obj_vel_sub;


Task_allocation* Task_allocator;

void (*chatterCallback_bases[N_ROBOTS])( const geometry_msgs::Pose &  ) =
		{chatterCallback_base0, chatterCallback_base1, chatterCallback_base2, chatterCallback_base3};

void (*chatterCallback_ends[N_ROBOTS])( const geometry_msgs::Pose &  ) =
		{chatterCallback_end0, chatterCallback_end1, chatterCallback_end2, chatterCallback_end3};


void (*chatterCallback_obj_pos[N_OBJECTS])( const geometry_msgs::Pose &  ) =
		{chatterCallback_obj_pos0, chatterCallback_obj_pos1, chatterCallback_obj_pos2, chatterCallback_obj_pos3};


void chatterCallback_obj_pos0( const geometry_msgs::Pose & msg)
{
	obj_state[0](0) = msg.position.x;
	obj_state[0](1) = msg.position.y;
	obj_state[0](2) = msg.position.z;
}

void chatterCallback_obj_pos1( const geometry_msgs::Pose & msg)
{
	obj_state[1](0) = msg.position.x;
	obj_state[1](1) = msg.position.y;
	obj_state[1](2) = msg.position.z;
}

void chatterCallback_obj_pos2( const geometry_msgs::Pose & msg)
{
	obj_state[2](0) = msg.position.x;
	obj_state[2](1) = msg.position.y;
	obj_state[2](2) = msg.position.z;
}

void chatterCallback_obj_pos3( const geometry_msgs::Pose & msg)
{
	obj_state[3](0) = msg.position.x;
	obj_state[3](1) = msg.position.y;
	obj_state[3](2) = msg.position.z;
}

void chatterCallback_end0( const geometry_msgs::Pose & msg)
{
	ends[0](0) =msg.position.x;
	ends[0](1) =msg.position.y;
	ends[0](2) =msg.position.z;
}

void chatterCallback_end1( const geometry_msgs::Pose & msg)
{
	ends[1](0) =msg.position.x;
	ends[1](1) =msg.position.y;
	ends[1](2) =msg.position.z;
}

void chatterCallback_end2( const geometry_msgs::Pose & msg)
{
	ends[2](0) =msg.position.x;
	ends[2](1) =msg.position.y;
	ends[2](2) =msg.position.z;
}

void chatterCallback_end3( const geometry_msgs::Pose & msg)
{
	ends[3](0) =msg.position.x;
	ends[3](1) =msg.position.y;
	ends[3](2) =msg.position.z;
}



void chatterCallback_base0( const geometry_msgs::Pose & msg)
{
	bases[0](0) =msg.position.x;
	bases[0](1) =msg.position.y;
	bases[0](2) =msg.position.z;
}

void chatterCallback_base1( const geometry_msgs::Pose & msg)
{
	bases[1](0) =msg.position.x;
	bases[1](1) =msg.position.y;
	bases[1](2) =msg.position.z;
}

void chatterCallback_base2( const geometry_msgs::Pose & msg)
{
	bases[2](0) =msg.position.x;
	bases[2](1) =msg.position.y;
	bases[2](2) =msg.position.z;
}

void chatterCallback_base3( const geometry_msgs::Pose & msg)
{
	bases[3](0) =msg.position.x;
	bases[3](1) =msg.position.y;
	bases[3](2) =msg.position.z;
}



int main(int argc, char **argv) {

	ros::init(argc, argv, "talker");

	init();

	init_topics();

	// while we haven't received everything, wait..

	// setup the task allocator
	while(ros::ok())
	{

	}
}

void init()
{
	Vector3d zeroVec;
	zeroVec.setZero();
	for(int i = 0; i < N_ROBOTS; i++)
	{
		bases.push_back(zeroVec);
		ends.push_back(zeroVec);
	}

	for(int i = 0; i < N_OBJECTS; i++)
	{
		VectorXd zero;
		zero.resize(n_state); zero.setZero();
		obj_state.push_back(zero);
		dobj_state.push_back(zero);
	}

}


void init_topics()
{
	std::ostringstream oss;

	for(int i = 0; i < N_ROBOTS; i++)
	{
		oss.clear();
		oss << "/robotsPat/targetID/" << i;
		rob_target_id_pub.push_back(n.advertise<std_msgs::Int64>(oss.str(), 1));


		oss.clear();
		oss << "/coordinationPat/" << i;
		rob_coordination_pub.push_back(n.advertise<std_msgs::Int64>(oss.str(), 1));

		oss.clear();
		oss << "/robot/base/" << i;
		//sub_velocity_object = n->subscribe("/object/filtered/velocity", 3, & Bi_manual_scenario::chatterCallback_ObjectVelocity,this);
		rob_end_sub.push_back(n.subscribe(oss.str(), 1, chatterCallback_bases[i]));

		oss.clear();
		oss << "/robot_real/end" << i;
		rob_base_sub.push_back(n.subscribe(oss.str(), 1, chatterCallback_ends[i]));
	}

	for(int i = 0; i < N_OBJECTS; i++)
	{

		oss.clear();
		oss << "/object/p" << i << "/position";
		obj_pos_sub.push_back(n.subscribe(oss.str(), 1, chatterCallback_obj_pos[i]));
	//	obj_vel_sub.push_back();
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


	Task_allocator->clear_coalitions();
}


void add_objects_task_allocator()
{
	cout << "adding objects" << endl;
	VectorXd X_O_G[N_grabbing];
	for(int i = 0; i < N_grabbing; i++)
	{
		X_O_G[i] = Object_Grabbing_State[i] - Object_State_raw;
	}


	double weight = 3.14;
	double value = 15;
	VectorXd single_grab[1];
	single_grab[0].resize(n_state); single_grab[0].setZero();

	Object task0(n_state,Object_State_raw,DObject_State, Task_allocator->get_max_time(), Task_allocator->get_dt(), Object_Grabbing_State, 2, weight, value, 0);
	Object task1(n_state,Object_State_raw,DObject_State, Task_allocator->get_max_time(), Task_allocator->get_dt(), single_grab, 1, weight*0.3, value, 1);
	Object task2(n_state,Object_State_raw,DObject_State, Task_allocator->get_max_time(), Task_allocator->get_dt(), single_grab, 1, weight*0.3, value, 2);
	Object task3(n_state,Object_State_raw,DObject_State, Task_allocator->get_max_time(), Task_allocator->get_dt(), single_grab, 1, weight*0.3, value, 3);


	Task_allocator->add_task(task0);
	Task_allocator->add_task(task1);
	Task_allocator->add_task(task2);
	Task_allocator->add_task(task3);

}


void add_robots_task_allocator()
{

	cout << "adding robots" << endl;
	double force = 5;
	int grippers = 1;

	for(int i = 0; i < 2; i++)
	{

		Vector3d base;
		base(0) = X[i];
		base(1) = Y[i];
		base(2) = Z[i];
		Robot_agent Bot(1,addTwochar(Commom_path,"/A_Matrix").c_str(), addTwochar(Commom_path,"/Priors").c_str(),
						addTwochar(Commom_path,"/Mu").c_str(), addTwochar(Commom_path,"/Sigma").c_str(),
						6, 3, addTwochar(Commom_path,"/IIWA_workspace_Model_prior").c_str(),
						addTwochar(Commom_path,"/IIWA_workspace_Model_mu").c_str(),
						addTwochar(Commom_path,"/IIWA_workspace_Model_Sigma").c_str(),
						addTwochar(Commom_path,"/IIWA_workspace_Model_Threshold").c_str(), base, i, grippers, force);

		Task_allocator->add_robot(Bot);

	//	cout << "robot " << i << " base " << base << endl;
	}
	//Task_allocator->print_bases();
}
