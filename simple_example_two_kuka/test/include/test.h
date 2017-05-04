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

#ifndef test_H_
#define test_H_


#include "RobotLib/RobotInterface.h"
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/JointState.h"
#include "kuka_fri_bridge/JointStateImpedance.h"
#include "qp_ik_solver.h"
#include "sKinematics.h"
#include "mathlib_eigen_conversions.h"
#include "PATH.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "sg_filter.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float64MultiArray.h"


const int KUKA_DOF=7;
int IK_CONSTRAINTS=9;
double IK_ORIENTATIONCONTROLSTART=0.01;
double Gain_velocity_limit = 3; //3 on the real robot
double Gain = 500;
SGF::real sample_time = 0.002;
double X[2];
double Y[2];
double Z[2];
enum ENUM_AXIS{AXIS_X=0, AXIS_Y, AXIS_Z};
enum ENUM_PLANNER{PLANNER_CARTESIAN=0, PLANNER_JOINT,PLANNER_NONE};




int order = 3;
int winlen = 5;




class test : public RobotInterface
{
public:
            test();
    virtual ~test();
  
    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
protected :
	void 						chatterCallback_left_position(const sensor_msgs::JointState & msg);
	void 						chatterCallback_right_position(const sensor_msgs::JointState & msg);
	void 						Send_Postion_To_Robot(int index,VectorXd Position);


	void 						Topic_initialization();
	void 						Parameter_initialization();
	void						initKinematics(int index);
	void 						initKinematics();
	void						prepare_solve_IK(int index);
	void						reset_the_bool();
	bool						everythingisreceived();
	void						pubish_on_tf(Vector3d  X,Quaterniond  Q,std::string n);
	void						prepare_jacobian(int index);
	void 						prepare_motion_generator(int index);





	bool						flag_init[2];
	bool						flag_job;
	bool						Position_of_the_robot_recieved[N_robots];


	VectorXd					cJob[N_robots];


	ros::Publisher 				pub_command_robot[N_robots];
	ros::Publisher 				pub_command_robot_real[N_robots];

    ros::Publisher              pub_gamma;
    std_msgs::Float64           msg_gamma;


	ros::Subscriber 			sub_position_robot[N_robots];



	tf::TransformBroadcaster		*tf_br;
	tf::Transform 					tf_transform;
	tf::Quaternion 					tf_q;

	VectorXd 					JointPos [N_robots];
	VectorXd 					JointVel [N_robots];
	VectorXd 					Desired_JointPos[N_robots];
	VectorXd 					JointDesVel[N_robots];


	VectorXd 					End_State [N_robots];
	VectorXd 					DEnd_State [N_robots];
	VectorXd 					Desired_End_State[N_robots];
	VectorXd 					Desired_DEnd_State[N_robots];

	Quaterniond					O_Robots;

	Vector3d					lDirX[N_robots];
	Vector3d					lDirY[N_robots];
	Vector3d					lDirZ[N_robots];
	Vector3d					RPos_End[N_robots];
	Vector3d					DRPos_End[N_robots];





	Matrix4d					T0[N_robots];

	Jacobian_S					Jacobian_R[N_robots];

	Matrix3d 					Robot_grasp[N_robots];
	Vector3d					Desired_DirX[N_robots];
	Vector3d					Desired_DirY[N_robots];
	Vector3d					Desired_DirZ[N_robots];
	Vector3d					Desired_Pos_End[N_robots];
	VectorXd 					Desired_Velocity[N_robots];



	sKinematics                 *mSKinematicChain[N_robots];
	qp_ik_solver				*IK_Solver;
	double						lDirWeight;

	MatrixXd					Jacobian3[N_robots];
	MatrixXd					Jacobian9[N_robots];
	MatrixXd					lJacobianDirY[N_robots];
	MatrixXd					lJacobianDirZ[N_robots];


	ENUM_COMMAND 				mCommand;
	ENUM_PLANNER 				mPlanner;

	SGF::SavitzkyGolayFilter 	*filter_pos_robot[N_robots];
	SGF::Vec 					inp;
	SGF::Vec 					outp;
	int 						ret_code;

};



#endif 
