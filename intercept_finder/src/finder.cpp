/*
  Copyright (c) 2016 Sina Mirrazavi,
  LASA Lab, EPFL, CH-1015 Lausanne, Switzerland,
  http://lasa.epfl.ch

  The program is free for non-commercial academic use.
  Please acknowledge the authors in any academic publications that have
  made use of this code or part of it.
  }
 */
#include "finder.h"



void chatterCallback_ObjectPosition_raw(const geometry_msgs::Pose & msg)
{
	P_object_raw(0)=msg.position.x;
	P_object_raw(1)=msg.position.y;
	P_object_raw(2)=msg.position.z;
	O_object_raw(0)=msg.orientation.x;
	O_object_raw(1)=msg.orientation.y;
	O_object_raw(2)=msg.orientation.z;
	O_object_raw(3)=msg.orientation.w;
	Position_of_the_object_recieved[N_grabbing+1]=true;
}
void chatterCallback_sub_G_On_object0(const geometry_msgs::Pose & msg)
{
	P_G_On_object[0](0)=msg.position.x;
	P_G_On_object[0](1)=msg.position.y;
	P_G_On_object[0](2)=msg.position.z;
	O_G_On_object[0](0)=msg.orientation.x;
	O_G_On_object[0](1)=msg.orientation.y;
	O_G_On_object[0](2)=msg.orientation.z;
	O_G_On_object[0](3)=msg.orientation.w;
	Position_of_the_object_recieved[0]=true;
	pubish_on_tf(P_G_On_object[0],O_G_On_object[0],addTwostring("Grabbing","On_object",0));

}
void chatterCallback_sub_G_On_object1(const geometry_msgs::Pose & msg)
{
	P_G_On_object[1](0)=msg.position.x;
	P_G_On_object[1](1)=msg.position.y;
	P_G_On_object[1](2)=msg.position.z;
	O_G_On_object[1](0)=msg.orientation.x;
	O_G_On_object[1](1)=msg.orientation.y;
	O_G_On_object[1](2)=msg.orientation.z;
	O_G_On_object[1](3)=msg.orientation.w;
	Position_of_the_object_recieved[1]=true;
	pubish_on_tf(P_G_On_object[1],O_G_On_object[1],addTwostring("Grabbing","On_object",1));

}
void chatterCallback_ObjectPosition(const geometry_msgs::Pose & msg)
{
	P_object_filtered(0)=msg.position.x;
	P_object_filtered(1)=msg.position.y;
	P_object_filtered(2)=msg.position.z;
	Position_of_the_object_recieved[N_grabbing]=true;
}
void chatterCallback_base_robot_0(const geometry_msgs::Pose & msg)
{
	X_base[0]=msg.position.x;
	Y_base[0]=msg.position.y;
	Z_base[0]=msg.position.z;
	cout<<"Base of robot 0"<<X_base[0]<<" "<<Y_base[0]<<" "<<Z_base[0]<<endl;
	Position_of_the_robot_recieved[0]=true;
}
void chatterCallback_base_robot_1(const geometry_msgs::Pose & msg)
{
	X_base[1]=msg.position.x;
	Y_base[1]=msg.position.y;
	Z_base[1]=msg.position.z;
	cout<<"Base of robot 1"<<X_base[1]<<" "<<Y_base[1]<<" "<<Z_base[1]<<endl;
	Position_of_the_robot_recieved[1]=true;
}
void chatterCallback_end_robot_0(const geometry_msgs::Pose & msg)
{
	X_end[0]=msg.position.x;
	Y_end[0]=msg.position.y;
	Z_end[0]=msg.position.z;
	cout<<"End of robot 0"<<X_end[0]<<" "<<Y_end[0]<<" "<<Z_end[0]<<endl;
	Position_of_the_robot_recieved[2]=true;
}
void chatterCallback_end_robot_1(const geometry_msgs::Pose & msg)
{
	X_end[1]=msg.position.x;
	Y_end[1]=msg.position.y;
	Z_end[1]=msg.position.z;
	cout<<"End of robot 1"<<X_end[1]<<" "<<Y_end[1]<<" "<<Z_end[1]<<endl;
	Position_of_the_robot_recieved[3]=true;
}


void chatterCallback_Command(const std_msgs::Int64& msg)
{
	int command;

	command=msg.data;

	switch(command){
	case COMMAND_INITIAL:
		cout<<"COMMAND_INITIAL"<<endl;
		mCommand=COMMAND_INITIAL;
		Motion_G= new multiarm_ds();
		Motion_G->Initialize_motion_predication_only(N_robots,N_grabbing,50*dt,6);
		reset_the_bool();
		while ((ros::ok())&&(!everythingisreceived()))
		{
			//cout<<"waiting for data"<<endl;
			ros::spinOnce();
		}
		cout<<"data is received"<<endl;
		for (int i=0;i<N_robots;i++)
		{

			Vector3d X_base_v;
			X_base_v(0)=X_base[i];
			X_base_v(1)=Y_base[i];
			X_base_v(2)=Z_base[i];
			Motion_G->Initialize_robot(i,1,addTwochar(Commom_path,"/A_Matrix").c_str(),addTwochar(Commom_path,"/Priors").c_str(),addTwochar(Commom_path,"/Mu").c_str(),addTwochar(Commom_path,"/Sigma").c_str(),
					6,3,addTwochar(Commom_path,"/IIWA_workspace_Model_prior").c_str(),addTwochar(Commom_path,"/IIWA_workspace_Model_mu").c_str(),addTwochar(Commom_path,"/IIWA_workspace_Model_Sigma").c_str(),addTwochar(Commom_path,"/IIWA_workspace_Model_Threshold").c_str(),X_base_v);
			X_base_v(0)=X_end[i];
			X_base_v(1)=Y_end[i];
			X_base_v(2)=Z_end[i];
			Motion_G->Set_the_initial_robot_state(i,X_base_v);
		}
		Motion_G->Set_the_object_state_for_prediction(P_object_raw,P_object_filtered,ros::Time::now().toSec());

		Object_State_raw.setZero();
		Object_State_raw.block(0,0,3,1)=P_object_raw;
		for(int i=0;i<N_grabbing;i++)
		{
			Object_Grabbing_State[i].setZero();
			Object_Grabbing_State[i].block(0,0,3,1)=P_G_On_object[i];
		}
		for (int i=0;i<N_grabbing;i++)
		{
			Motion_G->Set_the_grabbing_state(i,Object_Grabbing_State[i],Object_State_raw);
		}
		cout<<"Motion_G->Initialize is done"<<endl;
		cout<<"P_object_raw "<<P_object_raw<<endl;
		break;
	case COMMAND_JOB:
		cout<<"COMMAND_JOB"<<endl;
		mCommand=COMMAND_JOB;
		break;
	case COMMAND_Grab:
		cout<<"COMMAND_Grab"<<endl;
		mCommand=COMMAND_Grab;
		break;
	}
}


/*void Thread_prediction()
{


}*/


int main(int argc, char **argv) {

	// patrick below


		VectorXd X_p;	X_p.resize(3); X_p.setZero();//X = {1, 2, 3};
		VectorXd DX_p; DX_p.resize(3); DX_p.setOnes();// DX = {3,2,1};
		double max_time_p = 5.1;
		VectorXd grabbing_states_p[1];
		grabbing_states_p[0].resize(3); grabbing_states_p[0].setOnes();//grabbing_states[0] = {5,5,5};
		cout << "I'm about to make an Object" << endl;
		Object test(3, X_p, DX_p, max_time_p, grabbing_states_p, 1, 0.8, 130.0);
		cout << "made the object" << endl;
		cout << test << endl;


	// Communication service with robot module

	mCommand=COMMAND_NONE;


	Object_State_raw.resize(6);
	for (int i=0; i<N_grabbing;i++)
	{
		Object_Grabbing_State[i].resize(6);
	}

	ros::init(argc, argv, "prediction");
	ros::NodeHandle n;
	tf_br= new tf::TransformBroadcaster();
	sub_position_object_raw	= n.subscribe("/object/raw/position", 3, chatterCallback_ObjectPosition_raw);
	sub_position_object = n.subscribe("/object/filtered/position", 3, chatterCallback_ObjectPosition);
	sub_G_On_object[0]  = n.subscribe("/object/filtered/left/position", 3, chatterCallback_sub_G_On_object0);
	sub_G_On_object[1]  =n.subscribe("/object/filtered/right/position", 3, chatterCallback_sub_G_On_object1);
	sub_base_of_robot[0]= n.subscribe("/robot/base/0", 3, chatterCallback_base_robot_0);
	sub_base_of_robot[1]= n.subscribe("/robot/base/1", 3, chatterCallback_base_robot_1);
	sub_end_of_robot[0]= n.subscribe("/robot/end/0", 3, chatterCallback_end_robot_0);
	sub_end_of_robot[1]= n.subscribe("/robot/end/1", 3, chatterCallback_end_robot_1);
	sub_command = n.subscribe("/command", 3, chatterCallback_Command);


	pub_pos_prediction[0] = n.advertise<sensor_msgs::PointCloud>("/object/left/pointcloud", 3);
	pub_pos_prediction[1] = n.advertise<sensor_msgs::PointCloud>("/object/right/pointcloud", 3);

	pub_pos_catching = n.advertise<std_msgs::Float64MultiArray>("/catching/states", 3);

	//the_thread = new std::thread(Thread_prediction);

	ros::Rate r(100);
	initial_time=ros::Time::now().toSec();
	while (ros::ok())
	{
		ros::spinOnce();
		switch(mCommand){
		case COMMAND_INITIAL:
			break;
		case COMMAND_JOB:

			Motion_G= new multiarm_ds();
			break;
		case COMMAND_Grab:
			Motion_G->Set_the_object_state_for_prediction(P_object_filtered,P_object_filtered,ros::Time::now().toSec()-initial_time);
			test.set_prediction_state(P_object_filtered,P_object_filtered,ros::Time::now().toSec()-initial_time); // patrick
	//		cout << "object test X_O" <<  test.get_X_O() << endl;
			Motion_G->Set_the_object_state_for_prediction(P_object_raw,P_object_raw,ros::Time::now().toSec()-initial_time);
			if (Motion_G->Get_prediction_state())
			{
				Motion_G->predict_the_object_position();
				test.predict_motion(); // patrick


			//	cout << "Motion G" << endl <<  Motion_G->Get_P_O_Prediction() << endl << endl << endl;
				//cout << "Object " << endl << test.get_P_O_prediction() << endl << endl << endl;


				for (int i=0; i<N_grabbing;i++)
				{
					Motion_G->Get_predict_the_object_position(i,predicted_object[i]);
					pubish_on_point_cloud(i,predicted_object[i]);
				}
				for(int i=0;i<N_robots;i++)
				{
					Motion_G->Get_index_of_grabbing_posititon_(i,index_of_grabbing[i],Intercept_point_State[i]);
					pubish_on_tf(Intercept_point_State[i].block(0,0,3,1),O_object_raw,addTwostring("Desired","intercept_point",i));
				}
				if (Motion_G->Get_catching_state())
				{
					bool state=Motion_G->Get_pos_of_grabbing_posititon_for_object_(likelihood, X_I_C);
					int index_of_grabbing=-2;
					msg_float.data.resize(1+1+3+1+1+3+1+1+3);
					// 0 indicates the catching state
					// 1 indicates likelihood of catching
					// 2-3-4 indicates index of the catching position
					// 5 indicates index of the robot
					// 6 indicates index of the grabbing position for the 1 st robot
					// 7-8-9 indicates index of the catching position
					// 10 indicates index of the robot
					// 11 indicates index of the grabbing position for the 1 st robot
					// 12-13-14 indicates index of the catching position
					msg_float.data[0]=state;
					msg_float.data[1]=likelihood;
					msg_float.data[2]=X_I_C(0);	msg_float.data[3]=X_I_C(1);	msg_float.data[4]=X_I_C(2);
					index_of_grabbing=-2;

					Motion_G->Get_index_of_grabbing_posititon_(0,index_of_grabbing,X_I_C);
					msg_float.data[5]=0; msg_float.data[6]=index_of_grabbing;
					msg_float.data[7]=X_I_C(0);	msg_float.data[8]=X_I_C(1);	msg_float.data[9]=X_I_C(2);

					Motion_G->Get_index_of_grabbing_posititon_(1,index_of_grabbing,X_I_C);
					msg_float.data[10]=1; msg_float.data[11]=index_of_grabbing;
					msg_float.data[12]=X_I_C(0);	msg_float.data[13]=X_I_C(1);	msg_float.data[14]=X_I_C(2);					
					pub_pos_catching.publish(msg_float);
/*					for (int i=0;i<1+1+3+1+1+3+1+1+3;i++)
					{
						if (std::isnan(msg_float.data[i]))
						{
							return 0;
						}
					}*/
				}


			}
			break;
		}
		r.sleep();
		ros::spinOnce();
	}
	return 0;
}
