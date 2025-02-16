/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "Bi_manual_scenario.h"


/* The right robot (KUKA 7) is 1 and the left robot (KUKA 14) is 0*/


bool True_robot=false;

bool Using_target_moving = false;




ros::Publisher pub2;
ros::Publisher pub3;


// inverted by patrick, was object0 - primitive 1 and object1-primitive0

void Bi_manual_scenario::chatterCallback_sub_target_object0(const geometry_msgs::Pose & msg)
{
	Pfirst_primitive[0](0)=msg.position.x;
	Pfirst_primitive[0](1)=msg.position.y;
	Pfirst_primitive[0](2)=msg.position.z;
	T_G_On_object[0].x()=msg.orientation.x;
	T_G_On_object[0].y()=msg.orientation.y;
	T_G_On_object[0].z()=msg.orientation.z;
	T_G_On_object[0].w()=msg.orientation.w;
    Ofirst_primitive[0]=T_G_On_object[0].toRotationMatrix();
//	cout << "received primitive for robot 0 " << endl << Pfirst_primitive[0] << endl;
}
void Bi_manual_scenario::chatterCallback_sub_target_object1(const geometry_msgs::Pose & msg)
{
	Pfirst_primitive[1](0)=msg.position.x;
	Pfirst_primitive[1](1)=msg.position.y;
	Pfirst_primitive[1](2)=msg.position.z;
	T_G_On_object[1].x()=msg.orientation.x;
	T_G_On_object[1].y()=msg.orientation.y;
	T_G_On_object[1].z()=msg.orientation.z;
	T_G_On_object[1].w()=msg.orientation.w;
    Ofirst_primitive[1]=T_G_On_object[1].toRotationMatrix();
//	cout << "received primitive for robot 1 " << endl << Pfirst_primitive[1] << endl;
}


void Bi_manual_scenario::chatterCallback_sub_target_object2(const geometry_msgs::Pose & msg)
{
	Pfirst_primitive[2](0)=msg.position.x;
	Pfirst_primitive[2](1)=msg.position.y;
	Pfirst_primitive[2](2)=msg.position.z;
//	T_G_On_object[2].x()=msg.orientation.x;
//	T_G_On_object[2].y()=msg.orientation.y;
//	T_G_On_object[2].z()=msg.orientation.z;
//	T_G_On_object[2].w()=msg.orientation.w;

    Ofirst_primitive[2]=T_G_On_object[0].toRotationMatrix(); // careful here, the orientation still uses
//	cout << "received primitive for robot 2 " << endl << Pfirst_primitive[2] << endl;
}


void Bi_manual_scenario::chatterCallback_sub_target_object3(const geometry_msgs::Pose & msg)
{
	Pfirst_primitive[3](0)=msg.position.x;
	Pfirst_primitive[3](1)=msg.position.y;
	Pfirst_primitive[3](2)=msg.position.z;
//	T_G_On_object[3].x()=msg.orientation.x;
//	T_G_On_object[3].y()=msg.orientation.y;
//	T_G_On_object[3].z()=msg.orientation.z;
//	T_G_On_object[3].w()=msg.orientation.w;
    Ofirst_primitive[3]=T_G_On_object[1].toRotationMatrix();
//	cout << "received primitive for robot 3 " << endl << Pfirst_primitive[3] << endl;
}


void Bi_manual_scenario::chatterCallback_catching_state(const std_msgs::Float64MultiArray & msg)
{
	// 0 indicates the catching state
	// 1 indicates likelihood of catching
	// 2-3-4 indicates index of the catching position
	// 5 indicates index of the robot
	// 6 indicates index of the grabbing position for the 1 st robot
	// 7-8-9 indicates index of the catching position
	// 10 indicates index of the robot
	// 11 indicates index of the grabbing position for the 1 st robot
	// 12-13-14 indicates index of the catching position
	if (mPlanner==PLANNER_CARTESIAN)
	{
		/*			if (msg.data[1]>0.0001)
			{*/
		handle_of_likelihood=msg.data[1];
		//	}

		Vector3d X_I_C;
		X_I_C(0)=msg.data[2];	X_I_C(1)=msg.data[3];	X_I_C(2)=msg.data[4];
		Motion_G->Set_pos_of_grabbing_posititon_for_object_(msg.data[0],handle_of_likelihood, X_I_C);
		X_I_C(0)=msg.data[7];	X_I_C(1)=msg.data[8];	X_I_C(2)=msg.data[9];
		Motion_G->Set_index_of_grabbing_posititon_(msg.data[5],msg.data[6],X_I_C);
		X_I_C(0)=msg.data[12];	X_I_C(1)=msg.data[13];	X_I_C(2)=msg.data[14];
		Motion_G->Set_index_of_grabbing_posititon_(msg.data[10],msg.data[11],X_I_C);

        Motion_G->Set_index_of_grabbing_posititon_(2,0,X_I_C);

        Motion_G->Set_index_of_grabbing_posititon_(3,1,X_I_C);
	}
}
void Bi_manual_scenario::chatterCallback_ObjectPosition_raw(const geometry_msgs::Pose & msg)
{
	P_object_raw(0)=msg.position.x;
	P_object_raw(1)=msg.position.y;
	P_object_raw(2)=msg.position.z;
	O_object_raw.x()=msg.orientation.x;
	O_object_raw.y()=msg.orientation.y;
	O_object_raw.z()=msg.orientation.z;
	O_object_raw.w()=msg.orientation.w;
	Position_of_the_object_recieved[N_grabbing+3]=true;

/*	P_objects[0](0)=msg.position.x;
	P_objects[0](1)=msg.position.y;
	P_objects[0](2)=msg.position.z;
	O_objects[0].x()=msg.orientation.x;
	O_objects[0].y()=msg.orientation.y;
	O_objects[0].z()=msg.orientation.z;
	O_objects[0].w()=msg.orientation.w;*/

    publish_on_tf(P_object_raw,O_object_raw,"ObjectPosition_raw");
}


// patrick stuff
void Bi_manual_scenario::chatterCallback_rob0_coordination(const std_msgs::Float64 & msg)
{
	coordinations[0] = msg.data;
}

void Bi_manual_scenario::chatterCallback_rob1_coordination(const std_msgs::Float64 & msg)
{
	coordinations[1] = msg.data;
}

void Bi_manual_scenario::chatterCallback_rob2_coordination(const std_msgs::Float64 & msg)
{
	coordinations[2] = msg.data;
}

void Bi_manual_scenario::chatterCallback_rob3_coordination(const std_msgs::Float64 & msg)
{
	coordinations[3] = msg.data;
}


void Bi_manual_scenario::sendCommand(int _command)
{
	std_msgs::Int64 msg;

	msg.data=_command;

	pub_command.publish(msg);
}
void Bi_manual_scenario::chatterCallback_first_position(const sensor_msgs::JointState & msg)
{
	JointPos[0](0)=msg.position[0];
	JointPos[0](1)=msg.position[1];
	JointPos[0](2)=msg.position[2];
	JointPos[0](3)=msg.position[3];
	JointPos[0](4)=msg.position[4];
	JointPos[0](5)=msg.position[5];
	JointPos[0](6)=msg.position[6];
	Position_of_the_robot_recieved[0]=true;
}
void Bi_manual_scenario::chatterCallback_second_position(const sensor_msgs::JointState & msg)
{
	JointPos[1](0)=msg.position[0];
	JointPos[1](1)=msg.position[1];
	JointPos[1](2)=msg.position[2];
	JointPos[1](3)=msg.position[3];
	JointPos[1](4)=msg.position[4];
	JointPos[1](5)=msg.position[5];
	JointPos[1](6)=msg.position[6];
	Position_of_the_robot_recieved[1]=true;
}
void Bi_manual_scenario::chatterCallback_third_position(const sensor_msgs::JointState & msg)
{
	JointPos[2](0)=msg.position[0];
	JointPos[2](1)=msg.position[1];
	JointPos[2](2)=msg.position[2];
	JointPos[2](3)=msg.position[3];
	JointPos[2](4)=msg.position[4];
	JointPos[2](5)=msg.position[5];
	JointPos[2](6)=msg.position[6];
	Position_of_the_robot_recieved[2]=true;
}

void Bi_manual_scenario::chatterCallback_fourth_position(const sensor_msgs::JointState & msg)
{
	JointPos[3](0)=msg.position[0];
	JointPos[3](1)=msg.position[1];
	JointPos[3](2)=msg.position[2];
	JointPos[3](3)=msg.position[3];
	JointPos[3](4)=msg.position[4];
	JointPos[3](5)=msg.position[5];
	JointPos[3](6)=msg.position[6];
	Position_of_the_robot_recieved[3]=true;
}


void Bi_manual_scenario::chatterCallback_ObjectPosition(const geometry_msgs::Pose & msg)
{
	P_object(0)=msg.position.x;
	P_object(1)=msg.position.y;
	P_object(2)=msg.position.z;
	Position_of_the_object_recieved[N_grabbing]=true;
}
void Bi_manual_scenario::chatterCallback_ObjectVelocity(const geometry_msgs::Pose & msg)
{
	V_object(0)=msg.position.x;
	V_object(1)=msg.position.y;
	V_object(2)=msg.position.z;
	Position_of_the_object_recieved[N_grabbing+1]=true;
}
void Bi_manual_scenario::chatterCallback_ObjectAcceleration(const geometry_msgs::Pose & msg)
{
	A_object(0)=msg.position.x;
	A_object(1)=msg.position.y;
	A_object(2)=msg.position.z;
	Position_of_the_object_recieved[N_grabbing+2]=true;
}
void Bi_manual_scenario::chatterCallback_sub_G_On_object0(const geometry_msgs::Pose & msg)
{
	P_G_On_object[0](0)=msg.position.x;
	P_G_On_object[0](1)=msg.position.y;
	P_G_On_object[0](2)=msg.position.z;
	O_G_On_object[0].x()=msg.orientation.x;
	O_G_On_object[0].y()=msg.orientation.y;
	O_G_On_object[0].z()=msg.orientation.z;
	O_G_On_object[0].w()=msg.orientation.w;
	Position_of_the_object_recieved[0]=true;
    publish_on_tf(P_G_On_object[0],O_G_On_object[0],addTwostring("Grabbing","On_object",0));
	Object_ori[0]=O_G_On_object[0].toRotationMatrix();

	/*	Z_robot=Y_object
	 * 	Y_robot=X_object
	 * 	X_robot=Z_object*/

	Robot_grasp[0].block(0,0,3,1)=Object_ori[0].block(0,2,3,1);
	Robot_grasp[0].block(0,1,3,1)=Object_ori[0].block(0,0,3,1);
	Robot_grasp[0].block(0,2,3,1)=Object_ori[0].block(0,1,3,1);
	//	cout<<"Object_ori[0] "<<Object_ori[0]<<endl;

}
void Bi_manual_scenario::chatterCallback_sub_G_On_object1(const geometry_msgs::Pose & msg)
{
	P_G_On_object[1](0)=msg.position.x;
	P_G_On_object[1](1)=msg.position.y;
	P_G_On_object[1](2)=msg.position.z;
	O_G_On_object[1].x()=msg.orientation.x;
	O_G_On_object[1].y()=msg.orientation.y;
	O_G_On_object[1].z()=msg.orientation.z;
	O_G_On_object[1].w()=msg.orientation.w;
	Position_of_the_object_recieved[1]=true;
    publish_on_tf(P_G_On_object[1],O_G_On_object[1],addTwostring("Grabbing","On_object",1));
	Object_ori[1]=O_G_On_object[1].toRotationMatrix();

	/*	X_robot=-Z_object
	 * Y_robot=-X_object
	 * 	Z_robot=Y_object*/

	Robot_grasp[1].block(0,0,3,1)=-Object_ori[1].block(0,2,3,1);
	Robot_grasp[1].block(0,1,3,1)=-Object_ori[1].block(0,0,3,1);
	Robot_grasp[1].block(0,2,3,1)=Object_ori[1].block(0,1,3,1);

	//	cout<<"Object_ori[1] "<<Object_ori[1]<<endl;
}
void Bi_manual_scenario::chatterCallback_hand_state(const std_msgs::Int64 & msg)
{

	if (msg.data==0)
	{
		State=S_Open;
	}
	else if (msg.data==1)
	{
		State=S_Closed;
	}

}
void Bi_manual_scenario::Send_Postion_To_Robot(int index,VectorXd Position)
{
	kuka_fri_bridge::JointStateImpedance msg;
	std_msgs::Float64MultiArray msg_gazebo;


	msg.position.resize(KUKA_DOF);
	msg.stiffness.resize(KUKA_DOF);
	msg_gazebo.data.resize(KUKA_DOF);
	for (int i=0; i<KUKA_DOF;i=i+1)
	{
		msg.position[i]  = Position(i);
		msg.stiffness[i] = 2000;
		msg_gazebo.data[i]=Position(i);

	}

	pub_command_robot[index].publish(msg);

	if (True_robot)
	{
		pub_command_robot_real[index].publish(msg);
	}
	pub_command_robot_gazebo[index].publish(msg_gazebo);

//	if( index == 2)
//			cout << "sent robot " << index << " to position " << endl << Position << endl;

}
void Bi_manual_scenario::Topic_initialization()
{
	mRobot->SetControlMode(Robot::CTRLMODE_POSITION);
	ros::NodeHandle *n = mRobot->InitializeROS();



	if (True_robot)
	{
		sub_position_robot[0] = n->subscribe("/real_l_arm_pos_controller/joint_states", 3, & Bi_manual_scenario::chatterCallback_first_position,this);
		sub_position_robot[1]  = n->subscribe("/real_r_arm_pos_controller/joint_states", 3, & Bi_manual_scenario::chatterCallback_second_position,this);
	}
	else
	{
		for(int i = 0; i < N_robots; i++)
		{
			// this is an incredibly moronic way of doing it, the names are retarded and cant be automated..........
			if(i == 0)
			{
				sub_position_robot[i] = n->subscribe("/first_arm_pos_controller/joint_states", 3, & Bi_manual_scenario::chatterCallback_first_position,this);
				sub_target_of_robots[i]=n->subscribe("/robotsPat/target/0", 3, & Bi_manual_scenario::chatterCallback_sub_target_object0,this);
			}
			else if(i == 1)
			{
				sub_position_robot[i]  = n->subscribe("/second_arm_pos_controller/joint_states", 3, & Bi_manual_scenario::chatterCallback_second_position,this);
				sub_target_of_robots[i]=n->subscribe("/robotsPat/target/1", 3, & Bi_manual_scenario::chatterCallback_sub_target_object1,this);
			}
			else if(i == 2)
			{
				sub_position_robot[i] = n->subscribe("/third_arm_pos_controller/joint_states", 3, & Bi_manual_scenario::chatterCallback_third_position,this);
				sub_target_of_robots[i]=n->subscribe("/robotsPat/target/2", 3, & Bi_manual_scenario::chatterCallback_sub_target_object2,this);
			}
			else if(i == 3)
			{
				sub_position_robot[i]  = n->subscribe("/fourth_arm_pos_controller/joint_states", 3, & Bi_manual_scenario::chatterCallback_fourth_position,this);
				sub_target_of_robots[i]=n->subscribe("/robotsPat/target/3", 3, & Bi_manual_scenario::chatterCallback_sub_target_object3,this);
			}
		}
	}


	sub_position_object = n->subscribe("/object/filtered/position", 3, & Bi_manual_scenario::chatterCallback_ObjectPosition,this);
	sub_position_object_raw	= n->subscribe("/object/raw/position", 3, & Bi_manual_scenario::chatterCallback_ObjectPosition_raw,this);
	sub_velocity_object = n->subscribe("/object/filtered/velocity", 3, & Bi_manual_scenario::chatterCallback_ObjectVelocity,this);
	sub_acc_object		= n->subscribe("/object/filtered/acceleration", 3, & Bi_manual_scenario::chatterCallback_ObjectAcceleration,this);
	sub_G_On_object[0]  = n->subscribe("/object/filtered/left/position", 3, & Bi_manual_scenario::chatterCallback_sub_G_On_object0,this);
	sub_G_On_object[1]  = n->subscribe("/object/filtered/right/position", 3, & Bi_manual_scenario::chatterCallback_sub_G_On_object1,this);
	/*	sub_Orientation_On_object[0]  = n->subscribe("/object/raw/left/position", 3, & Bi_manual_scenario::chatterCallback_sub_G_On_object0,this);
	sub_Orientation_On_object[1]  = n->subscribe("/object/raw/left/position", 3, & Bi_manual_scenario::chatterCallback_sub_G_On_object0,this);*/
	sub_pos_catching	= n->subscribe("/catching/states", 3, & Bi_manual_scenario::chatterCallback_catching_state,this);



	// pat commented thse out and made the 2 below instead.
//	sub_traget_of_robots[0]=n->subscribe("/object/KUKA7_target/position", 3, & Bi_manual_scenario::chatterCallback_sub_target_object0,this);
//	sub_traget_of_robots[1]=n->subscribe("/object/KUKA14_target/position", 3, & Bi_manual_scenario::chatterCallback_sub_target_object1,this);
	// cant put these on a for loop because of the function pointer...


	for(int i = 0; i < N_robots; i++)
	{
		std::ostringstream oss;
		oss << "/robot/base/" << i;
		pub_base_of_robot[i] = n->advertise<geometry_msgs::Pose>(oss.str(), 3);

		oss.str("");
		oss.clear();
		oss << "/robot/end/" << i;
		pub_end_of_robot[i] = n->advertise<geometry_msgs::Pose>(oss.str(), 3);

		oss.str("");
		oss.clear();
		oss << "/robot_real/end/" << i;
		pub_end_of_robot_real[i] = n->advertise<geometry_msgs::Pose>(oss.str(), 3);

		oss.str("");
		oss.clear();
		oss << "/coordination/allocation/" << i;
		pub_allocation_robot[i] = n->advertise<std_msgs::Float64>(oss.str(), 3);


	/*	oss.str("");
		oss.clear();
		oss << "/coordination/allocation/" << i;
		pub_command_robot_gazebo[i] = n->advertise<std_msgs::Float64>(oss.str(), 3);
*/
	}

	for(int i = 0; i < N_robots; i++)
	{
		if(i == 0)
			pub_command_robot[i] = n->advertise<kuka_fri_bridge::JointStateImpedance>("/first_arm_controller/joint_imp_cmd", 3);
		else if(i == 1)
			pub_command_robot[i] = n->advertise<kuka_fri_bridge::JointStateImpedance>("/second_arm_controller/joint_imp_cmd", 3);
		else if(i == 2)
			pub_command_robot[i] = n->advertise<kuka_fri_bridge::JointStateImpedance>("/third_arm_controller/joint_imp_cmd", 3);
		else if(i == 3)
			pub_command_robot[i] = n->advertise<kuka_fri_bridge::JointStateImpedance>("/fourth_arm_controller/joint_imp_cmd", 3);
	}


	/*	if (True_robot)
	{*/
	for(int i = 0; i < N_robots; i++)
	{
		if(i == 0)
			pub_command_robot_real[i] =  n->advertise<kuka_fri_bridge::JointStateImpedance>("/first_arm_controller/joint_imp_cmd", 3);
		else if(i == 1)
			pub_command_robot_real[i] =  n->advertise<kuka_fri_bridge::JointStateImpedance>("/second_arm_controller/joint_imp_cmd", 3);
		else if(i == 2)
			pub_command_robot_real[i] =  n->advertise<kuka_fri_bridge::JointStateImpedance>("/third_arm_controller/joint_imp_cmd", 3);
		else if(i == 3)
			pub_command_robot_real[i] =  n->advertise<kuka_fri_bridge::JointStateImpedance>("/fourth_arm_controller/joint_imp_cmd", 3);
	}
	//	}


	pub_coordination_parameter= n->advertise<std_msgs::Float64>("/coordination/parameter", 3);

	for(int i = 0; i < N_robots; i++)
	{
		if(i == 0)
			pub_command_robot_gazebo[i] = n->advertise<std_msgs::Float64MultiArray>("/KUKA/Left/in", 3);
		else if(i == 1)
			pub_command_robot_gazebo[i] = n->advertise<std_msgs::Float64MultiArray>("/KUKA/Right/in", 3);
		else if(i == 2)
			pub_command_robot_gazebo[i] = n->advertise<std_msgs::Float64MultiArray>("/KUKA/backLeft/in", 3);
		else if(i == 3)
			pub_command_robot_gazebo[i] = n->advertise<std_msgs::Float64MultiArray>("/KUKA/backRight/in", 3);
	}






	sub_handState = n->subscribe("/Hand_state", 3, & Bi_manual_scenario::chatterCallback_hand_state,this);



	pub_pos_virtual = n->advertise<geometry_msgs::Pose>("/object/virtual/position", 3);


	pub_command = n->advertise<std_msgs::Int64>("/command", 3);

	pub_gamma= n->advertise<std_msgs::Float64>("/collision_avoidance/gamma", 3);
	tf_br= new tf::TransformBroadcaster();


	// patrick stuff
	sub_pat_coordination.clear();

	sub_pat_coordination.push_back(n->subscribe("/robotsPat/coordination/0", 3, &Bi_manual_scenario::chatterCallback_rob0_coordination, this));
    sub_pat_coordination.push_back(n->subscribe("/robotsPat/coordination/1", 3, &Bi_manual_scenario::chatterCallback_rob1_coordination, this));
	sub_pat_coordination.push_back(n->subscribe("/robotsPat/coordination/2", 3, &Bi_manual_scenario::chatterCallback_rob2_coordination, this));
	sub_pat_coordination.push_back(n->subscribe("/robotsPat/coordination/3", 3, &Bi_manual_scenario::chatterCallback_rob3_coordination, this));

}


void Bi_manual_scenario::Parameter_initialization()
{

	for(int i=0;i<N_robots;i++)
	{
		JointVel[i].resize(KUKA_DOF);			JointVel[i].setZero();
		Desired_Velocity[i].resize(9);			Desired_Velocity[i].setZero();
		JointDesVel[i].resize(KUKA_DOF);		JointDesVel[i].setZero();
		//JointPos_mirror[i].resize(KUKA_DOF);	JointPos_mirror[i].setZero();
		cJob[i].resize(KUKA_DOF);
	}

	for(int i=0;i<N_grabbing;i++)
	{
		Object_Grabbing_State[i].resize(6);
		Object_Grabbing_State[i].setZero();
		VirtualOb_Grabbing_State[i].resize(6);
		VirtualOb_Grabbing_State[i].setZero();
	}

	Object_State.resize(6); 		Object_State.setZero();
	Object_State_raw.resize(6); 	Object_State_raw.setZero();
	DObject_State.resize(6); 		DObject_State.setZero();

	VirtualOb_State.resize(6);	VirtualOb_State.setZero();

	A_V.resize(6,6);A_V.setZero();
	A_V(0,3)=1;
	A_V(1,4)=1;
	A_V(2,5)=1;
	A_V(3,0)=-1*Gain*Gain;		A_V(3,3)=-2*Gain;
	A_V(4,1)=-1*Gain*Gain;		A_V(4,4)=-2*Gain;
	A_V(5,2)=-1*Gain*Gain;		A_V(5,5)=-2*Gain;


	// Base offset of KUKA 14 for Bumper and Frame
	//	X[0]=0.143108;
	//	Y[0]=-1.43759;
	//	Z[0]=0.149693;

    // Base offset for KUKA 14 for Fender
//	X[0]=-0.0284889; // removed by patrick
//	Y[0]=-1.32351;
//	Z[0]=0.105877;

	X[0] = 0;
	Y[0] = 0;
	Z[0] = 0;


	// Base offset for KUKA 7
//	X[1]=0.0; // removed by patrick
//	Y[1]=0.0;
//	Z[1]=0.0;

	X[1] = 0;
	Y[1] = -1.25;
	Z[1] = 0;


    X[2] = 2;
	Y[2] = 0;
	Z[2] = 0;

    X[3] = 2;
	Y[3] = -1.25;
	Z[3] = 0;

/*	for(int i = 0; i < 2; i++)
	{
		X[i] = X[i+2];
		Y[i] = Y[i+2];
		Z[i] = Z[i+2];
	}
	*/

	// Desired Target for KUKA 14
    // was 0 0 1, 0 -1 0
    Desired_DirY[0](0)=0;		Desired_DirY[0](1)=0;			Desired_DirY[0](2)=1;
    Desired_DirZ[0](0)=0;		Desired_DirZ[0](1)=-1;			Desired_DirZ[0](2)=0;
    // Desired Target for KUKA 7
    Desired_DirY[1](0)=0;		Desired_DirY[1](1)=0;			Desired_DirY[1](2)=1;
    Desired_DirZ[1](0)=0;		Desired_DirZ[1](1)=1;			Desired_DirZ[1](2)=0;

    Desired_DirY[1](0)=0;		Desired_DirY[1](1)=0;			Desired_DirY[1](2)=1;
    Desired_DirZ[1](0)=0;		Desired_DirZ[1](1)=1;			Desired_DirZ[1](2)=0;

    // Desired Target for KUKA 14
    Desired_DirY[2](0)=0;		Desired_DirY[2](1)=0;			Desired_DirY[2](2)=1;
    Desired_DirZ[2](0)=0;		Desired_DirZ[2](1)=-1;			Desired_DirZ[2](2)=0;
    // Desired Target for KUKA 7
    Desired_DirY[3](0)=0;		Desired_DirY[3](1)=0;			Desired_DirY[3](2)=1;
    Desired_DirZ[3](0)=0;		Desired_DirZ[3](1)=1;			Desired_DirZ[3](2)=0;


	IK_Solver= new qp_ik_solver();
    IK_Solver->Initialize(2,dt, Numerical,Velocity_level, true, svm_filename); // 2 was first N_robots
	// Initialize IK Solver without Self-Collision Avoidance
	//IK_Solver->Initialize(N_robots,dt,Numerical,Velocity_level,true);

	// Initialize IK Solver with Self-Collision Avoidance

    IK_Solver_pat = new qp_ik_solver();
    IK_Solver_pat->Initialize(2,dt,Numerical,Velocity_level, true,  svm_filename);

	cout<<"IK_Solver->Initialize is done"<<endl;
	Motion_G= new multiarm_ds();
	Motion_G->Initialize(N_robots,N_grabbing,dt,6,A_V);
	cout<<"Motion_G->Initialize is done"<<endl;

    for(int i = 0 ; i < N_robots; i++)
    {
        cJob[i](0)=0.0;	cJob[i](1)=1.0472;	cJob[i](2)=0.0;	cJob[i](3)=-1.0472;	cJob[i](4)=0.0;	cJob[i](5)=-1.0192;	cJob[i](6)=0.0;
    }

	reset_the_bool();


}
void Bi_manual_scenario::initKinematics(int index)
{
	cout << "debug print, starting with kinematics of robot " << index << endl;
	if (index==0)
	{
		mSKinematicChain[index] = new sKinematics(KUKA_DOF, dt);

		/*
		mSKinematicChain[index]->setDH(0,  0.0,  0.36, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(98.0));
		mSKinematicChain[index]->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(98.0));
		mSKinematicChain[index]->setDH(2,  0.0,  0.42,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0));
		mSKinematicChain[index]->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(120.0));
		mSKinematicChain[index]->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0));
		mSKinematicChain[index]->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)); // reduced joint ang$
		//mSKinematicChain[index]->setDH(6,  0.0,  0.196, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		//	mSKinematicChain[index]->setDH(6,  0.0,  0.126+0.23, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
	//	mSKinematicChain[index]->setDH(6,  0.03,  0.126+0.14, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		mSKinematicChain[index]->setDH(6,  0.0,  0.1, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		 */

		mSKinematicChain[index]->setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(98.0));
		mSKinematicChain[index]->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(98.0));
		mSKinematicChain[index]->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0));
		mSKinematicChain[index]->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(130.0));
		mSKinematicChain[index]->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0));
		mSKinematicChain[index]->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)); // reduced joint ang$
		mSKinematicChain[index]->setDH(6,  0.0,  0.196, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		//	mSKinematicChain[index]->setDH(6,  0.0,  0.1, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$

	}
	else if  (index==1)
	{
		mSKinematicChain[index] = new sKinematics(KUKA_DOF, dt);

		mSKinematicChain[index]->setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(98.0));
		mSKinematicChain[index]->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(98.0));
		mSKinematicChain[index]->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0));
		mSKinematicChain[index]->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(120.0));
		mSKinematicChain[index]->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0));
		mSKinematicChain[index]->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)); // reduced joint ang$
		//mSKinematicChain[index]->setDH(6,  0.0,  0.196, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		//		mSKinematicChain[index]->setDH(6,  0.0,  0.126+14, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		mSKinematicChain[index]->setDH(6,  0.05,  0.126+0.11, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		//mSKinematicChain[index]->setDH(6,  0.00,  0.1, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
	}
	else if  (index==2)
	{
		mSKinematicChain[index] = new sKinematics(KUKA_DOF, dt);

		mSKinematicChain[index]->setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(98.0));
		mSKinematicChain[index]->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(98.0));
		mSKinematicChain[index]->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0));
		mSKinematicChain[index]->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(120.0));
		mSKinematicChain[index]->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0));
		mSKinematicChain[index]->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)); // reduced joint ang$
		//mSKinematicChain[index]->setDH(6,  0.0,  0.196, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		//		mSKinematicChain[index]->setDH(6,  0.0,  0.126+14, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		mSKinematicChain[index]->setDH(6,  0.05,  0.126+0.11, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		//mSKinematicChain[index]->setDH(6,  0.00,  0.1, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
	}
	else if  (index==3)
	{
		mSKinematicChain[index] = new sKinematics(KUKA_DOF, dt);

		mSKinematicChain[index]->setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(98.0));
		mSKinematicChain[index]->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(98.0));
		mSKinematicChain[index]->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0));
		mSKinematicChain[index]->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(120.0));
		mSKinematicChain[index]->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0));
		mSKinematicChain[index]->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)); // reduced joint ang$
		//mSKinematicChain[index]->setDH(6,  0.0,  0.196, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		//		mSKinematicChain[index]->setDH(6,  0.0,  0.126+14, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		mSKinematicChain[index]->setDH(6,  0.05,  0.126+0.11, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
		//mSKinematicChain[index]->setDH(6,  0.00,  0.1, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
	}



	T0[index].setZero();

	T0[index](0,0) = 1;
	T0[index](1,1) = 1;
	T0[index](2,2) = 1;
	T0[index](3,3) = 1;
	T0[index](0,3)=X[index];
	T0[index](1,3)=Y[index];
	T0[index](2,3)=Z[index];
	mSKinematicChain[index]->setT0(T0[index]);
	mSKinematicChain[index]->readyForKinematics();



	MatrixXd W(KUKA_DOF,KUKA_DOF); W.setZero();

	VectorXd U_p(KUKA_DOF);
	VectorXd U_Dp(KUKA_DOF);
	for(int i=0;i<KUKA_DOF;i++)
	{
		U_Dp(i)=Gain_velocity_limit*mSKinematicChain[index]->getMaxVel(i);
		U_p(i)=0.8*mSKinematicChain[index]->getMax(i);
	}


    if(index == 0 || index == 1)
		IK_Solver->Initialize_robot(index,KUKA_DOF,IK_CONSTRAINTS,W,U_p,-U_p,U_Dp,-U_Dp);
    else if(index == 2 || index == 3)
        IK_Solver_pat->Initialize_robot(index-2,KUKA_DOF,IK_CONSTRAINTS,W,U_p,-U_p,U_Dp,-U_Dp);

	// variable for ik
	Jacobian3[index].resize(3,KUKA_DOF);
	lJacobianDirY[index].resize(3,KUKA_DOF);
	lJacobianDirZ[index].resize(3,KUKA_DOF);
	Jacobian9[index].resize(9,KUKA_DOF);

	Vector3d X_base;
	X_base(0)=X[index];
	X_base(1)=Y[index];
	X_base(2)=Z[index];
	msg_robot_base.position.x=X[index];
	msg_robot_base.position.y=Y[index];
	msg_robot_base.position.z=Z[index];
	pub_base_of_robot[index].publish(msg_robot_base);
	pub_base_of_robot[index].publish(msg_robot_base);
	pub_base_of_robot[index].publish(msg_robot_base);
	pub_base_of_robot[index].publish(msg_robot_base);
	cout<<"Motion initialization for "<<index<<"the robot"<<endl;
	Motion_G->Initialize_robot(index,1,addTwochar(Common_path,"/A_Matrix").c_str(),addTwochar(Common_path,"/Priors").c_str(),addTwochar(Common_path,"/Mu").c_str(),addTwochar(Common_path,"/Sigma").c_str(),
			6,3,addTwochar(Common_path,"/IIWA_workspace_Model_prior").c_str(),addTwochar(Common_path,"/IIWA_workspace_Model_mu").c_str(),addTwochar(Common_path,"/IIWA_workspace_Model_Sigma").c_str(),addTwochar(Common_path,"/IIWA_workspace_Model_Threshold").c_str(),X_base);



	End_State[index].resize(6);		End_State[index].setZero();
	DEnd_State[index].resize(6);	DEnd_State[index].setZero();






	Desired_End_State[index].resize(6);		Desired_End_State[index].setZero();
	Desired_DEnd_State[index].resize(6);	Desired_DEnd_State[index].setZero();

	inp.resize(3);					outp.resize(3);
	RPos_End[index].setZero();

	filter_pos_robot[index]= new SGF::SavitzkyGolayFilter(3,order, winlen, sample_time);

	mSKinematicChain[index]->setJoints(JointPos[index].data());
	mSKinematicChain[index]->getEndPos(RPos_End[index]);
	msg_robot_end.position.x=RPos_End[index](0);
	msg_robot_end.position.y=RPos_End[index](1);
	msg_robot_end.position.z=RPos_End[index](2);
	pub_end_of_robot[index].publish(msg_robot_end);
	pub_end_of_robot[index].publish(msg_robot_end);
	pub_end_of_robot[index].publish(msg_robot_end);
	pub_end_of_robot[index].publish(msg_robot_end);
	Motion_G->Set_the_initial_robot_state(index,RPos_End[index]);



    for (int i=0;i<7;i++)
	{
		Jacobian_R[index].Jacobian[i].resize(3,1+i);Jacobian_R[index].Jacobian[i].setZero();
		Jacobian_R[index].Jacobian_7[i].resize(3,7);Jacobian_R[index].Jacobian_7[i].setZero();
	}
	//	Jacobian_R[index].Jacobian_Full.resize(3,28);Jacobian_R[index].Jacobian_Full.setZero();

	cout << "debug print, done with kinematics of robot " << index << endl;
}
void Bi_manual_scenario::prepare_solve_IK(int index)
{

	prepare_jacobian(index);
	mSKinematicChain[index]->getEndDirAxis(AXIS_Y, lDirY[index]);
	mSKinematicChain[index]->getEndDirAxis(AXIS_Z, lDirZ[index]);

	mSKinematicChain[index]->getJacobianPos(Jacobian3[index]);
	mSKinematicChain[index]->getJacobianDirection(AXIS_Y, lJacobianDirY[index]);
	mSKinematicChain[index]->getJacobianDirection(AXIS_Z, lJacobianDirZ[index]);



	/*
	if( (Desired_End_State[index].block(0,0,3,1)-RPos_End[index]).norm()> IK_ORIENTATIONCONTROLSTART ){
		lDirWeight = 0.0;
	}
	else{
		lDirWeight = (IK_ORIENTATIONCONTROLSTART-(Desired_End_State[index].block(0,0,3,1)-RPos_End[index]).norm())/IK_ORIENTATIONCONTROLSTART;
		if( lDirWeight > 1.0 ) lDirWeight = 1.0;
		else if(lDirWeight < 0.0 ) lDirWeight = 0.0;
	}
	 */
	lDirWeight=1;

	Jacobian9[index].block(0,0,3,KUKA_DOF)=Jacobian3[index];
	Jacobian9[index].block(3,0,3,KUKA_DOF)=lDirWeight*lJacobianDirY[index];
	Jacobian9[index].block(6,0,3,KUKA_DOF)=lDirWeight*lJacobianDirZ[index];


}
void Bi_manual_scenario::prepare_motion_generator(int index)
{
	mSKinematicChain[index]->setJoints(JointPos[index].data());
	mSKinematicChain[index]->getEndPos(RPos_End[index]);

	msg_robot_end.position.x=RPos_End[index](0);
	msg_robot_end.position.y=RPos_End[index](1);
	msg_robot_end.position.z=RPos_End[index](2);

	pub_end_of_robot_real[index].publish(msg_robot_end);

	inp(0) = RPos_End[index](0);
	inp(1) = RPos_End[index](1);
	inp(2) = RPos_End[index](2);
	ret_code = filter_pos_robot[index]->AddData(inp);
	ret_code = filter_pos_robot[index]->GetOutput(1, outp);
	DRPos_End[index](0)=outp(0);
	DRPos_End[index](1)=outp(1);
	DRPos_End[index](2)=outp(2);
	End_State[index].setZero();
	End_State[index].block(0,0,3,1)=RPos_End[index];
	//	End_State[index].block(3,0,3,1)=DRPos_End[index];
	End_State[index].block(3,0,3,1)=Desired_End_State[index].block(3,0,3,1);



	//	mSKinematicChain[index]->getEndDirAxis(AXIS_X, lDirX[index]);

}
void Bi_manual_scenario::reset_the_bool()
{
	for (int i=0;i<N_robots;i++)
	{
		Position_of_the_robot_recieved[i]=false;
	}
	for (int i=0;i<N_grabbing+4;i++)
	{
		Position_of_the_object_recieved[i]=false;
	}
}
bool Bi_manual_scenario::everythingisreceived()
{
	bool flag=true;

	for (int i=0;i<N_robots;i++)
	{

		if (Position_of_the_robot_recieved[i]==false)
		{
			cout<<"Position_of_the_robot_recieved[i] "<<i<<" "<<Position_of_the_robot_recieved[i]<<endl;
			flag=false;
		}
	}
	for (int i=0;i<N_grabbing+4;i++)
	{
		if (Position_of_the_object_recieved[i]==false)
		{
			cout<<"Position_of_the_object_recieved[i] "<<i<<" "<<Position_of_the_object_recieved[i]<<endl;
			flag=false;
		}
	}

	return flag;
}
/*void Bi_manual_scenario::publish_on_tf(Vector3d  X,Quaterniond  Q,std::string n)
{
	tf_transform.setOrigin( tf::Vector3(X(0),X(1), X(2)) );
	tf_q.setX(Q.x());tf_q.setY(Q.y());tf_q.setZ(Q.z());tf_q.setW(Q.w());
	tf_transform.setRotation(tf_q);
	tf_br->sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "world_frame", n));
}*/

void Bi_manual_scenario::publish_on_tf(VectorXd  X,Quaterniond  Q,std::string n)
{
	tf_transform.setOrigin( tf::Vector3(X(0),X(1), X(2)) );
	tf_q.setX(Q.x());tf_q.setY(Q.y());tf_q.setZ(Q.z());tf_q.setW(Q.w());
	tf_transform.setRotation(tf_q);
	tf_br->sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "world_frame", n));
}
void Bi_manual_scenario::prepare_jacobian(int index)
{
	for	(int i=0;i<7;i++)
	{

		mSKinematicChain[index]->getEndPos(i,Jacobian_R[index].Link_pos[i]);
		mSKinematicChain[index]->getJacobianPos_fast(i+1,Jacobian_R[index].Jacobian[i]);
		mSKinematicChain[index]->getJacobianPos(i+1,Jacobian_R[index].Jacobian_7[i]);
		/*		cout<<"Jacobian  of "<<index<<" robot. No "<< i<<"of jacobian"<<endl<<Jacobian_R[index].Jacobian[i]<<endl;
		cout<<"The full Jacobian  of "<<index<<" robot. No "<< i<<"of jacobian"<<endl<<Jacobian_R[index].Jacobian_7[i]<<endl;*/
	}
	Jacobian_R[index].Link_pos[0]=(Jacobian_R[index].Link_pos[0]+T0[index].block(0,3,3,1))/2;
	Jacobian_R[index].Link_pos[2]=(Jacobian_R[index].Link_pos[2]+Jacobian_R[index].Link_pos[1])/2;
	Jacobian_R[index].Link_pos[4]=(Jacobian_R[index].Link_pos[4]+Jacobian_R[index].Link_pos[3])/2;
	Jacobian_R[index].Link_pos[5]=(Jacobian_R[index].Link_pos[5]+Jacobian_R[index].Link_pos[6])/2;

	Jacobian_R[index].Jacobian[5]=Jacobian_R[index].Jacobian[6];
	Jacobian_R[index].Jacobian_7[5]=Jacobian_R[index].Jacobian_7[6];

}


/*void Bi_manual_scenario::publish_on_point_cloud(int index, MatrixXd  X)
{
	sensor_msgs::PointCloud 		pointcloud;

	pointcloud.points.resize(X.rows());
	pointcloud.header.stamp = ros::Time::now();
	pointcloud.header.frame_id="world_frame";
	for (int i=0;i<X.rows();i++)
	{
		pointcloud.points[i].x =X(i,0);
		pointcloud.points[i].y =X(i,1);
		pointcloud.points[i].z =X(i,2);
	}
	pub_pos_prediction[index].publish(pointcloud);
}*/



Bi_manual_scenario::Bi_manual_scenario()
:RobotInterface(){
	coordinations.push_back(0);
	coordinations.push_back(0);
	coordinations.push_back(0);
	coordinations.push_back(0);
}
Bi_manual_scenario::~Bi_manual_scenario(){
}

RobotInterface::Status Bi_manual_scenario::RobotInit(){
	for(int i=0;i<N_robots;i++)
	{
		Desired_JointPos[i].resize(KUKA_DOF);	Desired_JointPos[i].setZero();
		JointPos[i].resize(KUKA_DOF);			JointPos[i].setZero();
	}

	Topic_initialization();
	Parameter_initialization();
	for (int i=0;i<N_robots;i++)
	{
		initKinematics(i);
		Pfirst_primitive[i].resize(6);		Pfirst_primitive[i].setZero();
	}

	counter_Savinging=0;

	IK_Solver->Finalize_Initialization();
    IK_Solver_pat->Finalize_Initialization();
	mPlanner=PLANNER_NONE;
	mCommand=COMMAND_NONE;

	flag_job=true;
	AddConsoleCommand("init");
	AddConsoleCommand("job");
    AddConsoleCommand("catch");
    AddConsoleCommand("stop");





	return STATUS_OK;
}
RobotInterface::Status Bi_manual_scenario::RobotFree(){
	return STATUS_OK;
}
RobotInterface::Status Bi_manual_scenario::RobotStart(){

	while (!everythingisreceived())
	{
		ros::spinOnce();
	}


	for (int i=0;i<N_robots;i++)
	{
		Desired_JointPos[i]=JointPos[i];
		cout << "JointPos " << i << endl << JointPos[i] << endl;
	}
	Topic_initialization();



	return STATUS_OK;
}    
RobotInterface::Status Bi_manual_scenario::RobotStop(){
	return STATUS_OK;
}
RobotInterface::Status Bi_manual_scenario::RobotUpdate(){
	ros::spinOnce();

	switch(mCommand){
	case COMMAND_INITIAL :
		if (!flag_init[0])
		{
			sendCommand(COMMAND_INITIAL);
			cout<<"Initialization"<<endl;
			Parameter_initialization();
			cout<<"Parameter_initialization is done"<<endl;
			for (int i=0;i<N_robots;i++)
			{
				initKinematics(i);
			}
    //		cout<<"initKinematics is done"<<endl;
			IK_Solver->Finalize_Initialization();
            IK_Solver_pat->Finalize_Initialization();
    //		cout<<"Finalize_Initialization is done"<<endl;
			ros::spinOnce();
			flag_init[0]=true;
			reset_the_bool();
		}
		if (everythingisreceived()&&!flag_init[1])
		{
			VectorXd handle;handle.resize(6); handle.setZero();
			for(int i=0;i<N_robots;i++)
			{
				prepare_motion_generator(i);
                Motion_G->Set_the_robot_state(i,End_State[i]);
				if (Using_target_moving)
				{
					Motion_G->Set_coordination(i, coordinations[i]);
					Motion_G->Set_the_robot_first_primitive_desired_position(i,Pfirst_primitive[i],handle);

				}
				else
				{
					// pat modified here
				//	Motion_G->Set_the_robot_first_primitive_desired_position(i,End_State[i],handle);
					Motion_G->Set_coordination(i, coordinations[i]);
					Motion_G->Set_the_robot_first_primitive_desired_position(i,Pfirst_primitive[i],handle);
				}
				Desired_End_State[i]=End_State[i];
			}
			Object_State.block(0,0,3,1)=P_object;			Object_State.block(3,0,3,1)=V_object;
			Object_State_raw.block(0,0,3,1)=P_object_raw;	Object_State_raw.block(3,0,3,1)=V_object;
			DObject_State.block(0,0,3,1)=V_object;			DObject_State.block(3,0,3,1)=A_object;


			Motion_G->Set_the_object_state(Object_State_raw,DObject_State);



			for(int i=0;i<N_grabbing;i++)
			{
				Object_Grabbing_State[i].block(0,0,3,1)=P_G_On_object[i];
				Object_Grabbing_State[i].block(3,0,3,1)=V_object;
			}

			for (int i=0;i<N_grabbing;i++)
			{
				Motion_G->Set_the_grabbing_state(i,Object_Grabbing_State[i],Object_State_raw);
			}


			Motion_G->Initialize_the_virtual_object();
			Motion_G->Update();
			flag_init[1]=true;
			counter_Savinging=counter_Savinging+1;
			sprintf(Outputfile,"TheRobotdata_fender%d.txt",counter_Savinging);
			TheRobotdata.open(Outputfile);
			cout<<"Initialization finished"<<endl;
		}
		mPlanner=PLANNER_NONE;
		break;
	case COMMAND_JOB:
		sendCommand(COMMAND_JOB);
		for(int i=0;i<N_robots;i++)
		{
			prepare_motion_generator(i);
			prepare_jacobian(i);

			cout<<"RPos_End "<<i<<endl;cout<<RPos_End[i]<<endl;
		}
		mPlanner=PLANNER_JOINT;
		mCommand=COMMAND_NONE;
		break;
	case COMMAND_Grab:
		sendCommand(COMMAND_Grab);
		mPlanner=PLANNER_CARTESIAN;
		mCommand=COMMAND_NONE;
		break;
	}




	return STATUS_OK;
}
RobotInterface::Status Bi_manual_scenario::RobotUpdateCore(){

	switch(mPlanner){
	case PLANNER_CARTESIAN :

		for(int i=0;i<N_robots;i++)
		{
			prepare_motion_generator(i);
			// For closed loop
        //	Motion_G->Set_the_robot_state(i,End_State[i]);
			VectorXd handle;handle.resize(6); handle.setZero();
			if (Using_target_moving)
			{

				// removed by patrick VectorXd handle;handle.resize(6); handle.setZero();
				Motion_G->Set_coordination(i, coordinations[i]); // added patrick
				Motion_G->Set_the_robot_first_primitive_desired_position(i,Pfirst_primitive[i],handle);




				//	cout<<"Pfirst_primitive[i] "<<i<<endl<<Pfirst_primitive[i]<<endl;
			}


			Motion_G->Set_coordination(i, coordinations[i]); // added patrick
			Motion_G->Set_the_robot_first_primitive_desired_position(i,Pfirst_primitive[i],handle); // added patrick

//			cout << "robot " << i << " coordination " << coordinations[i] << " primitive " << endl << Pfirst_primitive[i] << endl;
//			cout << "set robot " << i << " coord to " << coordinations[i] << " and target " << endl << Pfirst_primitive[i] << endl;
			// For open loop
            Motion_G->Set_the_robot_state(i,Desired_End_State[i]);
		}
		if (State==S_Open)
		{
			for(int i=0;i<N_robots;i++)
			{
				TheRobotdata<<End_State[i](0)<<" "<<End_State[i](1)<<" "<<End_State[i](2)<<" ";
			}
			TheRobotdata<<P_object(0)<<" "<<P_object(1)<<" "<<P_object(2)<<" ";
		}

		Object_State.block(0,0,3,1)=P_object;			Object_State.block(3,0,3,1)=V_object;
		Object_State_raw.block(0,0,3,1)=P_object_raw;	Object_State_raw.block(3,0,3,1)=V_object;
		DObject_State.block(0,0,3,1)=V_object;			DObject_State.block(3,0,3,1)=A_object;

		Motion_G->Set_the_object_state(Object_State,DObject_State);

		Motion_G->Update();


		for(int i=0;i<N_robots;i++)
		{
			Motion_G->Get_the_coordination_allocation(i,msg_coordination.data);
			if (msg_coordination.data>0.1)
			{
				Robot_grasp_final[i]=Robot_grasp[i];
			}
			else
			{
				Robot_grasp_final[i]=Ofirst_primitive[i];
			}

			if (State==S_Open)
			{
				pub_allocation_robot[i].publish(msg_coordination);
			}
			if (State==S_Open)
			{
				TheRobotdata<<msg_coordination.data<<" ";
			}
		}
		Motion_G->Get_the_coordination_parameter(msg_coordination.data);
		if (State==S_Open)
		{
			pub_coordination_parameter.publish(msg_coordination);
		}
		if (State==S_Open)
		{
			TheRobotdata<<msg_coordination.data<<" ";
		}

		Motion_G->Get_Virtual_state(VirtualOb_State);
		if (State==S_Open)
		{
			TheRobotdata<<VirtualOb_State(0)<<" "<<VirtualOb_State(1)<<" "<<VirtualOb_State(2)<<" ";
		}
		msg_vobject.position.x=VirtualOb_State(0);msg_vobject.position.y=VirtualOb_State(1);msg_vobject.position.z=VirtualOb_State(2);
		pub_pos_virtual.publish(msg_vobject);
        publish_on_tf(VirtualOb_State.block(0,0,3,1),O_object_raw,"Virtual_object");
		for (int i=0;i<N_grabbing;i++)
		{
			Motion_G->Get_the_grabbing_state(i,VirtualOb_Grabbing_State[i]);
			O_object=Robot_grasp[i];
     //       ROS_INFO_STREAM("grabbing position " << i << " is at " << VirtualOb_Grabbing_State[i].block(0,0,3,1).transpose() << " with virtual object at " << VirtualOb_State.block(0,0,3,1).transpose());
            publish_on_tf(VirtualOb_Grabbing_State[i].block(0,0,3,1),O_object,addTwostring("Grabbing","On_virtual",i));
		}
    //	publish_on_tf(Pfirst_primitive[1],T_G_On_object[1],"First_primitive_KUKA7");
    //	publish_on_tf(Pfirst_primitive[0],T_G_On_object[0],"First_primitive_KUKA14");
		if (Motion_G->Get_catching_state())
		{

			for(int i=0;i<N_robots;i++)
			{

				Motion_G->Get_the_robot_state(i,Desired_End_State[i]);
         //       Desired_DirY[i]=Robot_grasp_final[i].block(0,1,3,1);
          //      Desired_DirZ[i]=Robot_grasp_final[i].block(0,2,3,1);
				O_object=Robot_grasp[i];

				if (State==S_Open)
				{
					TheRobotdata<<Desired_End_State[i](0)<<" "<<Desired_End_State[i](1)<<" "<<Desired_End_State[i](2)<<" ";
				}

                publish_on_tf(Desired_End_State[i].block(0,0,3,1),O_object,addTwostring("End","desired_position",i));
			}

			for(int i=0;i<N_robots;i++)
			{

				prepare_solve_IK(i);

                if(i == 0 || i == 1)
				{
      //              cout << "jacobian of robot " << i << " to old IK solver" << endl;
					IK_Solver->set_jacobian_links(i,Jacobian_R[i]);
					IK_Solver->set_jacobian(i,Jacobian9[i]);
				}
                else if(i == 2 || i == 3)
                {
       //             cout << "jacobian of robot " << i << " to pat IK solver" << endl;
                    IK_Solver_pat->set_jacobian_links(i-2,Jacobian_R[i]);
                    IK_Solver_pat->set_jacobian(i-2,Jacobian9[i]);
                }
              //  cout<<"i "<<i<<" "<<Desired_End_State[i].block(0,0,3,1)<<endl;
                Desired_Velocity[i].block(0,0,3,1)=(Desired_End_State[i].block(0,0,3,1)-RPos_End[i])/dt;
     //           cout<<"(Desired_End_State[i].block(0,0,3,1)-RPos_End[i])/dt "<<Desired_End_State[i].block(0,0,3,1)<<" i "<<i<<endl;

				/*	cout<<"Desired_End_State[i] "<<i<<endl<<Desired_End_State[i].block(0,0,3,1)<<endl;
					Desired_Velocity[i].block(0,0,3,1)=Desired_End_State[i].block(3,0,3,1);
					c*/

                Desired_Velocity[i].block(3,0,3,1)=(Desired_DirY[i]-lDirY[i])/(10*dt); // was 10
                Desired_Velocity[i].block(6,0,3,1)=(Desired_DirZ[i]-lDirZ[i])/(10*dt); // was 10
          //      cout << "trying to set desired state for robot " << i << endl;
                if(i == 0 || i == 1)
				{

					IK_Solver->set_desired(i,Desired_Velocity[i]);

					IK_Solver->set_state(i,JointPos[i],JointVel[i]);
				}

                else if(i == 2 || i == 3)
                {
                    IK_Solver_pat->set_desired(i-2,Desired_Velocity[i]);
                    IK_Solver_pat->set_state(i-2,JointPos[i],JointVel[i]);
                }
       //         cout << "done setting desired state for robot " << i << endl;
			}

  //          cout << "solving " << endl;
			IK_Solver->Solve();
//			if(N_robots > 1)
   //         cout << "solved IK" << endl;
            IK_Solver_pat->Solve();
			//	IK_Solver->Solve_QP();

  //          cout << "done solving " << endl;
			// Publishing Gamma to Visualize on RQT_PLOT FOR SELF-COLLISION AVOIDANCE
    //		IK_Solver->get_gamma(msg_gamma.data);
    //		pub_gamma.publish(msg_gamma);

			for(int i=0;i<N_robots;i++)
			{
				Desired_JointPos[i]=JointPos[i];
                if(i < 2)
					IK_Solver->get_state(i,JointDesVel[i]);
                else
                    IK_Solver_pat->get_state(i-2, JointDesVel[i]);
				Desired_JointPos[i]=Desired_JointPos[i]+JointDesVel[i]*dt;
				JointVel[i]=JointDesVel[i];
			}

		}
		if (State==S_Open)
		{
			TheRobotdata<<dt<<endl;
		}
		break;
	case PLANNER_JOINT:
		for(int i=0;i<N_robots;i++)
		{
			prepare_motion_generator(i);
			prepare_jacobian(i);
			Desired_JointPos[i]=JointPos[i]+100*(cJob[i]-JointPos[i])*dt;
		}
		break;
	}

	if (State==S_Open)
	{
		for(int i=0;i<N_robots;i++)
		{
			Send_Postion_To_Robot(i,Desired_JointPos[i]);
		}
	}
	else
	{
		cout<<" Hands are closed "<<endl;
		TheRobotdata.close();
	}

	return STATUS_OK;
}
int Bi_manual_scenario::RespondToConsoleCommand(const string cmd, const vector<string> &args){
	if(cmd=="init"){
		if (!flag_job)
		{
			flag_init[0]=false;
			flag_init[1]=false;
			mPlanner=PLANNER_NONE;
			mCommand = COMMAND_INITIAL;
		}
	}
	else if(cmd=="job"){
		mCommand = COMMAND_JOB;
		mPlanner=PLANNER_NONE;
		flag_job=false;
	}
	else if(cmd=="catch"){
		if (!flag_job)
		{
			cout<<"Catch!"<<endl;
			mCommand = COMMAND_Grab;
			mPlanner=PLANNER_NONE;
			flag_job=true;
		}
	}
    else if(cmd=="stop")
    {
        cout << "Stop/start the ball!" << endl;
        std_msgs::Int64 msg;
        msg.data = COMMAND_STOP;
        pub_command.publish(msg);
    }
	return 0;
}


extern "C"{
// These two "C" functions manage the creation and destruction of the class
Bi_manual_scenario* create(){return new Bi_manual_scenario();}
void destroy(Bi_manual_scenario* module){delete module;}
}

