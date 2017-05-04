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

#ifndef example_H_
#define example_H_

#include "RobotLib/RobotInterface.h"
#include "sensor_msgs/JointState.h"
#include "kuka_fri_bridge/JointStateImpedance.h"
#include "qp_ik_solver.h"
#include "sKinematics.h"
#include "mathlib_eigen_conversions.h"
#include "multiarm_ds.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "sg_filter.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float64MultiArray.h"
#include "commands.h"




const int KUKA_DOF=7;
int IK_CONSTRAINTS=9;
double IK_ORIENTATIONCONTROLSTART=0.01;
double Gain_velocity_limit=500; //50 on the real robot
double Gain=50;
SGF::real sample_time = 0.002;
double X[4];
double Y[4];
double Z[4];
enum ENUM_AXIS{AXIS_X=0, AXIS_Y, AXIS_Z};
enum ENUM_PLANNER{PLANNER_CARTESIAN=0, PLANNER_JOINT,PLANNER_NONE};



int order = 3;
int winlen = 5;



using namespace Eigen;
enum ENUM_Robot{Left, Right};

class Bi_manual_scenario : public RobotInterface
{
public:
	Bi_manual_scenario();
	virtual ~Bi_manual_scenario();

	virtual Status              RobotInit();
	virtual Status              RobotFree();

	virtual Status              RobotStart();
	virtual Status              RobotStop();

	virtual Status              RobotUpdate();
	virtual Status              RobotUpdateCore();

	virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
private :


	void						chatterCallback_rob0_coordination(const std_msgs::Float64 & msg);
	void						chatterCallback_rob1_coordination(const std_msgs::Float64 & msg);
	void						chatterCallback_rob2_coordination(const std_msgs::Float64 & msg);
	void						chatterCallback_rob3_coordination(const std_msgs::Float64 & msg);


	void 						chatterCallback_first_position(const sensor_msgs::JointState & msg);
	void 						chatterCallback_second_position(const sensor_msgs::JointState & msg);
	void 						chatterCallback_third_position(const sensor_msgs::JointState & msg);
	void 						chatterCallback_fourth_position(const sensor_msgs::JointState & msg);
	void						chatterCallback_ObjectPosition(const geometry_msgs::Pose & msg);
	void						chatterCallback_ObjectPosition_raw(const geometry_msgs::Pose & msg);
	void						chatterCallback_ObjectVelocity(const geometry_msgs::Pose & msg);
	void						chatterCallback_ObjectAcceleration(const geometry_msgs::Pose & msg);
	void						chatterCallback_sub_G_On_object0(const geometry_msgs::Pose & msg);
	void						chatterCallback_sub_G_On_object1(const geometry_msgs::Pose & msg);
	void						chatterCallback_sub_target_object0(const geometry_msgs::Pose & msg);
	void						chatterCallback_sub_target_object1(const geometry_msgs::Pose & msg);
	void						chatterCallback_catching_state(const std_msgs::Float64MultiArray & msg);
	void 						chatterCallback_hand_state(const std_msgs::Int64 & msg);
	void 						Send_Postion_To_Robot(int index,VectorXd Position);

	void						chatterCallback_sub_target_object2(const geometry_msgs::Pose & msg); // pat hack
	void						chatterCallback_sub_target_object3(const geometry_msgs::Pose & msg); // pat hack


	void						chatterCallback_ObjectPositionP1(const geometry_msgs::Pose & msg); // patrick
	void						chatterCallback_ObjectPositionP2(const geometry_msgs::Pose & msg); // patrick
	void						chatterCallback_ObjectPositionP3(const geometry_msgs::Pose & msg); // patrick




	void 						Topic_initialization();
	void 						Parameter_initialization();
	void						initKinematics(int index);
	void 						initKinematics();
	void						prepare_solve_IK(int index);
	void 						prepare_motion_generator(int index);
	void 						sendCommand(int _command);
	void						reset_the_bool();
	bool						everythingisreceived();
//	void						pubish_on_tf(Vector3d  X,Quaterniond  Q,std::string n);
	void						pubish_on_tf(VectorXd  X,Quaterniond  Q,std::string n);
	void						pubish_on_point_cloud(int index, MatrixXd  X);
	void						Thread_prediction( );
	void						prepare_jacobian(int index);


	bool						flag_init[2];
	bool						flag_job;
	bool						Position_of_the_robot_recieved[N_robots];
	bool						Position_of_the_object_recieved[N_grabbing+4];

	pthread_t					PredictionThread;

	VectorXd					cJob[N_robots];

	ros::Subscriber 			sub_handState;
	ros::Publisher 				pub_command_robot[N_robots];
	ros::Publisher 				pub_command_robot_real[N_robots];
	ros::Publisher 				pub_allocation_robot[N_robots];
	ros::Publisher 				pub_coordination_parameter;
	ros::Publisher 				pub_command_robot_gazebo[N_robots];
	ros::Publisher 				pub_base_of_robot[N_robots];
	ros::Publisher 				pub_end_of_robot[N_robots];
	ros::Publisher 				pub_end_of_robot_real[N_robots];
	ros::Publisher 				pub_pos_virtual;
	ros::Subscriber 			sub_position_object;
	ros::Subscriber 			sub_position_object_raw;
	ros::Subscriber 			sub_velocity_object;
	ros::Subscriber 			sub_acc_object;
	ros::Subscriber 			sub_position_robot[N_robots];
	ros::Subscriber 			sub_G_On_object[N_grabbing];
	ros::Subscriber 			sub_target_of_robots[N_robots];
/*	ros::Subscriber 			sub_Orientation_On_object[N_grabbing];*/
	ros::Subscriber 			sub_pos_catching;


	ros::Publisher 				pub_command;
	geometry_msgs::Pose			msg_vobject;
	geometry_msgs::Pose			msg_robot_base;
	geometry_msgs::Pose			msg_robot_end;
	std_msgs::Float64			msg_coordination;


	tf::TransformBroadcaster		*tf_br;
	tf::Transform 					tf_transform;
	tf::Quaternion 					tf_q;

	VectorXd 					JointPos [N_robots];
	VectorXd 					JointVel [N_robots];
	VectorXd 					Desired_JointPos[N_robots];
	VectorXd 					JointDesVel[N_robots];

/*	VectorXd					Objects_state[4]; // patrick
	Vector3d					P_objects[4];					// patrick
	Quaterniond					O_objects[4];					// patrick
*/
	VectorXd 					End_State [N_robots];
	VectorXd 					DEnd_State [N_robots];
	VectorXd 					Desired_End_State[N_robots];
	VectorXd 					Desired_DEnd_State[N_robots];


	Matrix4d					T0[N_robots];

	Vector3d					lDirX[N_robots];
	Vector3d					lDirY[N_robots];
	Vector3d					lDirZ[N_robots];
	Vector3d					RPos_End[N_robots];
	Vector3d					DRPos_End[N_robots];
	VectorXd 					Intercept_point_State[N_robots];		// Position of the desired intercept point

	VectorXd 					Object_State;
	VectorXd 					Object_State_raw;
	VectorXd 					Object_Grabbing_State[N_grabbing];
	VectorXd 					DObject_State;

	VectorXd 					VirtualOb_State;						// Position of the virtual object
	VectorXd 					VirtualOb_Grabbing_State[N_grabbing];	// Position of the grabbing positions on the virtual


	VectorXd					Pfirst_primitive[N_robots];

	Matrix3d					Ofirst_primitive[N_robots];
	Quaterniond					T_G_On_object[N_grabbing];		// Orientation of the grabbing positions on the object




	Vector3d					P_object_raw;					// Position of the object 	(raw)
	Quaterniond					O_object_raw;					// Orientation of the object(raw)
	Vector3d					P_object;						// Position of the object 	(filtered)
	Quaterniond					O_object; 						// Orientation of the object(filtered)
	Vector3d					V_object;						// Velocity of the object 	(filtered)
	Vector3d					A_object;						// Acceleration of the object 	(filtered)
	Vector3d					P_G_On_object[N_grabbing];		// Position of the grabbing positions on the object
	Quaterniond					O_G_On_object[N_grabbing];		// Orientation of the grabbing positions on the object
	Quaterniond					O_targets;


	Jacobian_S					Jacobian_R[N_robots];



	Vector3d					Desired_DirY[N_robots];
	Vector3d					Desired_DirZ[N_robots];
	Vector3d					Desired_Pos_End[N_robots];
	VectorXd 					Desired_Velocity[N_robots];



	sKinematics                 *mSKinematicChain[N_robots];
	qp_ik_solver				*IK_Solver;

	qp_ik_solver				*IK_Solver_pat;

	double						lDirWeight;

    ros::Publisher              pub_gamma;
    std_msgs::Float64           msg_gamma;


	MatrixXd					Jacobian3[N_robots];
	MatrixXd					Jacobian9[N_robots];
	MatrixXd					lJacobianDirY[N_robots];
	MatrixXd					lJacobianDirZ[N_robots];

	Matrix3d 					Object_ori[N_grabbing];
	Matrix3d 					Robot_grasp[N_robots];


	Matrix3d 					Robot_grasp_final[N_robots];



	MatrixXd					A_V;

	multiarm_ds					*Motion_G;

	double 						handle_of_likelihood;


	ENUM_COMMAND 				mCommand;
	ENUM_PLANNER 				mPlanner;



	SGF::SavitzkyGolayFilter 	*filter_pos_robot[N_robots];
	SGF::Vec 					inp;
	SGF::Vec 					outp;
	int 						ret_code;


	Status_hand State;


	char 						Outputfile[256];
	int							counter_Savinging;
	std::ofstream				TheRobotdata;


	// patrick stuff
	std::vector<ros::Subscriber> sub_pat_targets;
	std::vector<ros::Subscriber> sub_pat_coordination;

	std::vector<double> 		coordinations;
};



#endif 
