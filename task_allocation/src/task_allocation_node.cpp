#include "task_allocation_node.h"
#include <ctime>



const int n_state = 6;


bool run = false;
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

std::vector<Coalition> active_coalitions;


Task_allocation* Task_allocator;

void chatter_ready(const std_msgs::Int64 msg)
{
    if(msg.data == 0)
        run = 1;
    if(msg.data == 4)
    {
        if(run == false)
            run = true;
        else
            run = false;
    }

}


int main(int argc, char **argv) {

	MatrixXd targets;
	std::vector<double> coordinations;

	ros::init(argc, argv, "task_allocation");

	n = new ros::NodeHandle();

	init();

	prepare_task_allocator();

	init_topics();


    ROS_INFO_STREAM("done initializing task allocation" << endl);

    ROS_INFO_STREAM("allocating " << N_ROB << " robots to " << N_OBJ << " objects" << endl);
    ROS_INFO_STREAM("waiting for simulator start" << endl);
	// while we haven't received everything, wait.. This is not very orthodox

    while(run == false) // 0 is the value we get from the "init" in the "job init catch" sequence
	{
		if(ros::ok())
		{
			ros::spinOnce();
		}
	}

  /*  for(auto & rob : Robots)
    {

        cout << rob << endl;
    }*/

    cout << *Task_allocator << endl;
    ROS_INFO_STREAM("starting allocating in a loop" << endl);



	// loop...
    ros::Rate r(TASK_ALLOCATION_RATE);
	while(ros::ok())
    {

        if(run == true)
        {
      //      cout << *Task_allocator << endl;

            clock_t begin = clock();

     //       ROS_INFO_STREAM("Updating value" << endl);
            Task_allocator->update_objects_value();
    //        ROS_INFO_STREAM("predicting motion" << endl);
            Task_allocator->predict_motion();
   //         ROS_INFO_STREAM("updating robot business" << endl);
            Task_allocator->update_rob_business();
  //          ROS_INFO_STREAM("allocating" << endl);
            Task_allocator->allocate();
 //           ROS_INFO_STREAM("computing intercepts" << endl);
            Task_allocator->compute_intercepts(); // is actually done in "allocate()".
//            ROS_INFO_STREAM("computing coordination" << endl);
            Task_allocator->compute_coordination();
          //  ROS_INFO_STREAM("done" << endl);


            active_coalitions = Task_allocator->get_coalitions();
            // go through each coalition and see if the robots are where we want them
            for(auto & coal : active_coalitions)
            {
                std::vector<int> ids;
                ids = coal.get_robots_id();
                std::vector<bool> reached;
                double distance_POG;
                int n_grips = Objects[coal.get_object_id()].get_n_grippers(); // the "get_object_id() can theoretically return -1, but should not happen in active coalitions. If it happens, there's another error.
                for(int i = 0; i < ids.size(); i++)
                {
                    // for robot 0, check POG 0, for robot 1, check POG 1
                    distance_POG = (Robots[ids[i]].get_end() - Objects[coal.get_object_id()].get_P_O_G_prediction(i%2).col(0).block(0,0,3,1)).norm();
                    if(distance_POG < 0.25)
                        reached.push_back(true);
                    else
                        reached.push_back(false);
                }
                if(n_grips == 1)
                {
                    if(reached[0] == true)
                    {
                        // single robot in coalition and it caught the item
                        cout << "robot " << ids[0] << " grabbed object " << coal.get_object_id() << endl;
                        Robots[ids[0]].set_grabbed();
                        Robots[ids[0]].set_idle();
                    }
                }
                else if(n_grips == 2)
                {
                    if(reached[0] == true && reached[1] == true)
                    {
                        // 2 robots in coalition and they caught the item
                        for(auto & id : ids)
                        {
                            Robots[id].set_grabbed();
                        }
                        // Here I somehow need to set the virtual object going up...
                    }
                }
            }

            for(auto & rob : Robots)
            {
                if(rob.get_status() == Robot_status::Unallocated)
                {
                    rob.set_idle();
                }
                if(rob.has_grabbed())
                {
                    if((rob.get_idle_pos() - rob.get_end()).norm() < 0.1)
                    {
                        Objects[rob.get_assignment()].set_done();
                    }
                }
            }

/*
            // check at what state the robots are
            for(auto & rob : Robots)
            {
                int assign = rob.get_assignment();
                if(assign != -1) // if it's assigned
                {
                    int n_grips = Objects[assign].get_n_grippers();
                    if(n_grips == 1) // if its object is a "1 robot" object
                    {
                        double distance = (rob.get_end() - Objects[assign].get_X_O().block(0,0,3,1)).norm();
        //				cout << "robot " << rob.get_id() << " is at " << distance << " of its target" << endl;
                        if( distance < 0.2) // if it reached the object
                        {
           //                 cout << "robot " << rob.get_id() << " grabbed object " << Objects[assign].get_id() << endl;
                            rob.set_grabbed();
                        }
                    }

                    if(n_grips == 2)
                    {
                        double distance_POG[n_grips];
                        for(int i = 0; i < n_grips; i++)
                        {
                            distance_POG[i] = (rob.get_end() - Objects[assign].get_P_O_G_prediction(i).col(0).block(0,0,3,1)).norm();
                        }
                        if(distance_POG[0] < 0.15 && distance_POG[1] < 0.15)
                        {
                            ROS_INFO_STREAM("robot " << rob.get_id() << "distance to POG " << distance_POG[0] << " " << distance_POG[1]);

                        }
                    }
                }


                if(rob.has_grabbed()) // if the robot has grabbed something
                {
                    rob.set_idle(); // keep sending it to idle position

                    if((rob.get_idle_pos() - rob.get_end()).norm() < 0.1)
                    {
                        Objects[rob.get_assignment()].set_done();
         //               cout << "robot " << rob.get_id() << " released the object " << Objects[rob.get_assignment()].get_id() << endl;
                    }
                }
            }
*/
            clock_t end = clock();


            // publish stuff
            targets = Task_allocator->get_targets();

            coordinations = Task_allocator->get_coordinations();

            // publish who grabbed what.
            for(auto & rob : Robots)
            {
                std_msgs::Int64 msg;
                msg.data = -1;
                if(rob.has_grabbed())
                {
                    msg.data = rob.get_assignment();
                }
                rob_grabbed_pub[rob.get_id()].publish(msg);
            }
     /*       for(auto & rob : Robots)
            {
                int assign = rob.get_assignment();
                std_msgs::Int64 msg;
                msg.data = -1;
                if(assign >= 0)
                {
                    if(Objects[assign].get_n_grippers() == 1)
                    {
                        if(rob.has_grabbed())
                            msg.data = assign;

                    }
                }

                rob_grabbed_pub[rob.get_id()].publish(msg);
            }
*/

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
            for(int i = 0; i < N_ROB; i++)
            {
                rob_id_msg.data = Task_allocator->get_robot_target(i);
                rob_target_id_pub[i].publish(rob_id_msg);
            }


            for(int i = 0; i < N_OBJ; i++)
            {
                std_msgs::Int64 msg;
                msg.data = Objects[i].is_done();
                obj_done_pub[i].publish(msg);

            }

            clock_t end_pub = clock();
        }

        ros::spinOnce();
        r.sleep();


        // warning if we're too slow
        if(r.cycleTime().toNSec() > r.expectedCycleTime().toNSec())
        {
            ROS_INFO_STREAM("Task allocation node is not reaching it's desired frequency! Cycletime " << r.cycleTime() << " expected cycletime " << r.expectedCycleTime());
        }
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
    double_grab[0].resize(n_state); double_grab[0].setZero();
    double_grab[1].resize(n_state); double_grab[1].setZero();
    double_grab[0](1) = -0.1;
    double_grab[1](1) = 0.1;


    for(int i = 0; i < N_OBJ; i++)
    {
        if(obj_sizes[(int)(SCENARIO)][i] == Object_sizes::LARGE) // shouldn't cast this to int. The clean way is making a switch statement but breaks the modularity of the code.
            Objects.push_back(Object(n_state,zeroVec,zeroVec, Task_allocator->get_max_time(), Task_allocator->get_dt(), double_grab, 2, weight, value, i));
        else
            Objects.push_back(Object(n_state,zeroVec,zeroVec, Task_allocator->get_max_time(), Task_allocator->get_dt(), single_grab, 1, weight*0.3, value, i));
    }



/*	Objects.push_back(task0);
	Objects.push_back(task1);
	Objects.push_back(task2);
	Objects.push_back(task3);
*/

	for(auto & obj : Objects)
		Task_allocator->add_task(&obj);
}


void add_robots_task_allocator()
{

	cout << "adding robots" << endl;
	double force = 5;
	int grippers = 1;

    for(int i = 0; i < N_ROB; i++)
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
