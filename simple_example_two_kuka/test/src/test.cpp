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

#include "test.h"


bool True_robot=true;

void test::chatterCallback_left_position(const sensor_msgs::JointState & msg)
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
void test::chatterCallback_right_position(const sensor_msgs::JointState & msg)
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

void test::Send_Postion_To_Robot(int index,VectorXd Position)
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



}
void test::Topic_initialization()
{
	mRobot->SetControlMode(Robot::CTRLMODE_POSITION);
	ros::NodeHandle *n = mRobot->InitializeROS();



	if (True_robot)
	{
		sub_position_robot[0] = n->subscribe("/real_l_arm_pos_controller/joint_states", 3, & test::chatterCallback_left_position,this);
		sub_position_robot[1]  = n->subscribe("/real_r_arm_pos_controller/joint_states", 3, & test::chatterCallback_right_position,this);
	}
	else
	{
		sub_position_robot[0] = n->subscribe("/l_arm_pos_controller/joint_states", 3, & test::chatterCallback_left_position,this);
		sub_position_robot[1]  = n->subscribe("/r_arm_pos_controller/joint_states", 3, & test::chatterCallback_right_position,this);
	}


	pub_command_robot[0] = n->advertise<kuka_fri_bridge::JointStateImpedance>("/l_arm_controller/joint_imp_cmd", 3);
	pub_command_robot[1] = n->advertise<kuka_fri_bridge::JointStateImpedance>("/r_arm_controller/joint_imp_cmd", 3);


	pub_command_robot_real[0] =  n->advertise<kuka_fri_bridge::JointStateImpedance>("/real_l_arm_controller/joint_imp_cmd", 3);
	pub_command_robot_real[1] =  n->advertise<kuka_fri_bridge::JointStateImpedance>("/real_r_arm_controller/joint_imp_cmd", 3);

    pub_gamma= n->advertise<std_msgs::Float64>("/collision_avoidance/gamma", 3);
	tf_br= new tf::TransformBroadcaster();

}
void test::Parameter_initialization()
{

	for(int i=0;i<N_robots;i++)
	{
		JointVel[i].resize(KUKA_DOF);			JointVel[i].setZero();
		Desired_Velocity[i].resize(9);			Desired_Velocity[i].setZero();
		JointDesVel[i].resize(KUKA_DOF);		JointDesVel[i].setZero();
		//JointPos_mirror[i].resize(KUKA_DOF);	JointPos_mirror[i].setZero();
		cJob[i].resize(KUKA_DOF);
	}


    // Base offset for KUKA 14
	X[0]=-0.000225067;
	Y[0]=-1.30596;
	Z[0]=0.149834;

    // Base offset for KUKA 7
	X[1]=0.0;
    Y[1]=0.0;
	Z[1]=0.0;

    // Desired Target for KUKA 14
    Desired_DirX[0](0)=1;		Desired_DirX[0](1)=0;			Desired_DirX[0](2)=0;
    Desired_DirY[0](0)=0;		Desired_DirY[0](1)=1;			Desired_DirY[0](2)=0;
    Desired_DirZ[0](0)=0;		Desired_DirZ[0](1)=0;			Desired_DirZ[0](2)=1;
	Robot_grasp[0]<<Desired_DirX[0],Desired_DirY[0],Desired_DirZ[0];    
    Desired_Pos_End[0](0)=0.0; 	Desired_Pos_End[0](1)=-0.60; Desired_Pos_End[0](2)=0.55;

    // Desired Target for KUKA 7
    Desired_DirX[1](0)=1;		Desired_DirX[1](1)=0;			Desired_DirX[1](2)=0;
    Desired_DirY[1](0)=0;		Desired_DirY[1](1)=0;			Desired_DirY[1](2)=1;
    Desired_DirZ[1](0)=0;		Desired_DirZ[1](1)=-1;			Desired_DirZ[1](2)=0;
	Robot_grasp[1]<<Desired_DirX[1],Desired_DirY[1],Desired_DirZ[1];
    Desired_Pos_End[1](0)=0.0;    Desired_Pos_End[1](1)=-0.65; 	Desired_Pos_End[1](2)=0.85;


    // Initialize IK Solver with Self-Collision Avoidance
	IK_Solver= new qp_ik_solver();
    IK_Solver->Initialize(N_robots,dt,Numerical,Velocity_level, true, svm_filename);
	cout<<"IK_Solver->Initialize is done"<<endl;


	cJob[0](0)=0.0;	cJob[0](1)=-1.0;	cJob[0](2)=0.0;	cJob[0](3)=-1.57;	cJob[0](4)=0.0;	cJob[0](5)=-0.57;	cJob[0](6)=0.0;

	if (N_robots>1)
	{
		cJob[1](0)=0.0;	cJob[1](1)=-1.0;	cJob[1](2)=0.0;	cJob[1](3)=-1.57;	cJob[1](4)=0.0;	cJob[1](5)=-0.57;	cJob[1](6)=0.0;
	}    
	reset_the_bool();

}
void test::initKinematics(int index)
{

	mSKinematicChain[index] = new sKinematics(KUKA_DOF, dt);

	mSKinematicChain[index]->setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(98.0));
	mSKinematicChain[index]->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(98.0));
	mSKinematicChain[index]->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0));
	mSKinematicChain[index]->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(130.0));
	mSKinematicChain[index]->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0));
	mSKinematicChain[index]->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)); // reduced joint ang$
	//mSKinematicChain[index]->setDH(6,  0.0,  0.196, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$
	mSKinematicChain[index]->setDH(6,  0.0,  0.1, 0.0, 	 0.0, 1,  DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0)); // reduced joint ang$

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
		U_p(i)=mSKinematicChain[index]->getMax(i);
	}


	IK_Solver->Initialize_robot(index,KUKA_DOF,IK_CONSTRAINTS,W,U_p,-U_p,U_Dp,-U_Dp);

	// variable for ik
	Jacobian3[index].resize(3,KUKA_DOF);
	lJacobianDirY[index].resize(3,KUKA_DOF);
	lJacobianDirZ[index].resize(3,KUKA_DOF);
	Jacobian9[index].resize(9,KUKA_DOF);

	End_State[index].resize(6);		End_State[index].setZero();
	DEnd_State[index].resize(6);	DEnd_State[index].setZero();


	Desired_End_State[index].resize(6);		Desired_End_State[index].setZero();
	Desired_DEnd_State[index].resize(6);	Desired_DEnd_State[index].setZero();

	RPos_End[index].setZero();

	filter_pos_robot[index]= new SGF::SavitzkyGolayFilter(3,order, winlen, sample_time);


	mSKinematicChain[index]->setJoints(JointPos[index].data());
	mSKinematicChain[index]->getEndPos(RPos_End[index]);



	for (int i=0;i<7;i++)
	{
		Jacobian_R[index].Jacobian[i].resize(3,1+i);Jacobian_R[index].Jacobian[i].setZero();
		Jacobian_R[index].Jacobian_7[i].resize(3,7);Jacobian_R[index].Jacobian_7[i].setZero();
	}
	//	Jacobian_R[index].Jacobian_Full.resize(3,28);Jacobian_R[index].Jacobian_Full.setZero();
}
void test::prepare_solve_IK(int index)
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
	lDirWeight=1	;

	Jacobian9[index].block(0,0,3,KUKA_DOF)=Jacobian3[index];
	Jacobian9[index].block(3,0,3,KUKA_DOF)=lDirWeight*lJacobianDirY[index];
	Jacobian9[index].block(6,0,3,KUKA_DOF)=lDirWeight*lJacobianDirZ[index];

}
void test::prepare_motion_generator(int index)
{
	mSKinematicChain[index]->setJoints(JointPos[index].data());
	mSKinematicChain[index]->getEndPos(RPos_End[index]);


	//	mSKinematicChain[index]->getEndDirAxis(AXIS_X, lDirX[index]);

}
void test::reset_the_bool()
{
	for (int i=0;i<N_robots;i++)
	{
		Position_of_the_robot_recieved[i]=false;
	}
}
bool test::everythingisreceived()
{
	bool flag=true;

	for (int i=0;i<N_robots;i++)
	{
		//		cout<<"Position_of_the_robot_recieved[i] "<<i<<" "<<Position_of_the_robot_recieved[i]<<endl;
		if (Position_of_the_robot_recieved[i]==false)
		{
			flag=false;
		}
	}

	return flag;
}
void test::pubish_on_tf(Vector3d  X,Quaterniond  Q,std::string n)
{
	tf_transform.setOrigin( tf::Vector3(X(0),X(1), X(2)) );
	tf_q.setX(Q.x());tf_q.setY(Q.y());tf_q.setZ(Q.z());tf_q.setW(Q.w());
	tf_transform.setRotation(tf_q);
	tf_br->sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "world_frame", n));
}

void test::prepare_jacobian(int index)
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

}



test::test()
:RobotInterface(){
}
test::~test(){
}

RobotInterface::Status test::RobotInit(){
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
	}

	IK_Solver->Finalize_Initialization();

	mPlanner=PLANNER_NONE;
	mCommand=COMMAND_NONE;

	flag_job=true;
	AddConsoleCommand("init");
	AddConsoleCommand("job");
	AddConsoleCommand("catch");
	AddConsoleCommand("Target_Pos");
	return STATUS_OK;
}
RobotInterface::Status test::RobotFree(){
	return STATUS_OK;
}
RobotInterface::Status test::RobotStart(){

	while (!everythingisreceived())
	{
		ros::spinOnce();
	}

	for (int i=0;i<N_robots;i++)
	{
		Desired_JointPos[i]=JointPos[i];
	}
	cout<<"JointPos_left"<<endl;cout<<JointPos[0]<<endl;
	cout<<"JointPos_right"<<endl;cout<<JointPos[1]<<endl;

	Topic_initialization();
	return STATUS_OK;
}    
RobotInterface::Status test::RobotStop(){
	return STATUS_OK;
}
RobotInterface::Status test::RobotUpdate(){
	ros::spinOnce();

	switch(mCommand){
	case COMMAND_INITIAL :
		if (!flag_init[0])
		{
			cout<<"Initialization"<<endl;
			Parameter_initialization();
			cout<<"Parameter_initialization is done"<<endl;
			for (int i=0;i<N_robots;i++)
			{
				initKinematics(i);
			}
			cout<<"initKinematics is done"<<endl;
			IK_Solver->Finalize_Initialization();
			cout<<"Finalize_Initialization is done"<<endl;
			ros::spinOnce();
			flag_init[0]=true;
			reset_the_bool();
		}
		if (everythingisreceived()&&!flag_init[1])
		{
			VectorXd handle;handle.resize(6); handle.setZero();
			for(int i=0;i<N_robots;i++)
			{
				Desired_End_State[i]=End_State[i];
			}
			flag_init[1]=true;
			cout<<"Initialization finished"<<endl;
		}
		mPlanner=PLANNER_NONE;
		break;
	case COMMAND_JOB:
		for(int i=0;i<N_robots;i++)
		{
			cout<<"RPos_End "<<i<<endl;cout<<RPos_End[i]<<endl;
		}
		mPlanner=PLANNER_JOINT;
		mCommand=COMMAND_NONE;
		break;
	case COMMAND_Grab:
		mPlanner=PLANNER_CARTESIAN;
		mCommand=COMMAND_NONE;
		break;
	}




	return STATUS_OK;
}
RobotInterface::Status test::RobotUpdateCore(){

	switch(mPlanner){
	case PLANNER_CARTESIAN :

		for(int i=0;i<N_robots;i++)
		{
			prepare_motion_generator(i);
		}
		for(int i=0;i<N_robots;i++)
		{
			Desired_DirY[i]=Robot_grasp[i].block(0,1,3,1);
			Desired_DirZ[i]=Robot_grasp[i].block(0,2,3,1);

			O_Robots=Robot_grasp[i];

			pubish_on_tf(Desired_Pos_End[i].block(0,0,3,1),O_Robots,addTwostring("End","desired_position",i));
		}

		for(int i=0;i<N_robots;i++)
		{

			prepare_solve_IK(i);
			IK_Solver->set_jacobian_links(i,Jacobian_R[i]);
			IK_Solver->set_jacobian(i,Jacobian9[i]);
			Desired_Velocity[i].block(0,0,3,1)=(Desired_Pos_End[i]-RPos_End[i])/dt;
//            cout<<"Desired_Pos_End[i] "<<i<<endl<<Desired_Pos_End[i]<<endl;
//            cout<<"RPos_End "<<RPos_End[i]<<endl;
			Desired_Velocity[i].block(3,0,3,1)=(Desired_DirY[i]-lDirY[i])/dt;
			Desired_Velocity[i].block(6,0,3,1)=(Desired_DirZ[i]-lDirZ[i])/dt;
			IK_Solver->set_desired(i,Desired_Velocity[i]);
			IK_Solver->set_state(i,JointPos[i],JointVel[i]);
		}



		IK_Solver->Solve();

        // Publishing Gamma to Visualize on RQT_PLOT
        IK_Solver->get_gamma(msg_gamma.data);
        pub_gamma.publish(msg_gamma);

		for(int i=0;i<N_robots;i++)
		{
			Desired_JointPos[i]=JointPos[i];
			IK_Solver->get_state(i,JointDesVel[i]);
			Desired_JointPos[i]=Desired_JointPos[i]+JointDesVel[i]*dt;
			JointVel[i]=JointDesVel[i];
		}
		/*		while (ros::ok)
		{

		}*/
		break;
	case PLANNER_JOINT:
		for(int i=0;i<N_robots;i++)
		{
			prepare_motion_generator(i);
			prepare_jacobian(i);
			Desired_JointPos[i]=JointPos[i]+5*(cJob[i]-JointPos[i])*dt;
		}
		break;
	}

	for(int i=0;i<N_robots;i++)
	{
		Send_Postion_To_Robot(i,Desired_JointPos[i]);
	}
	return STATUS_OK;
}
int test::RespondToConsoleCommand(const string cmd, const vector<string> &args){
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
	else if (cmd=="Target_Pos")
	{
		if (atof(args[0].c_str())==0)
		{
			Desired_Pos_End[0](0)=atof(args[1].c_str()); 			Desired_Pos_End[0](1)=atof(args[2].c_str());			Desired_Pos_End[0](2)=atof(args[3].c_str());
		}
		else if (atof(args[0].c_str())==1)
		{
			Desired_Pos_End[1](0)=atof(args[1].c_str()); 			Desired_Pos_End[1](1)=atof(args[2].c_str());			Desired_Pos_End[1](2)=atof(args[3].c_str());
		}
	}
	return 0;
}



extern "C"{
// These two "C" functions manage the creation and destruction of the class
test* create(){return new test();}
void destroy(test* module){delete module;}
}
