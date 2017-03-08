
#include "ros/ros.h"
#include "ros/service.h"
#include "ros/service_server.h"
#include "sensor_msgs/JointState.h"
#include "CDDynamics.h"
#include "MathLib.h"
#include  "std_msgs/Int64.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"

#include "PATH.h"

#define JOINT_STATE_TOPIC "/allegro/joint_state"
#define JOINT_CMD_TOPIC "/allegro/joint_cmd"

enum Command{Com_hand_INIT=0, Com_hand_Move,Com_hand_non};

enum Status_hand{S_Open=0, S_Closed};


Status_hand State;
Command COM;

ros::Subscriber sub_command;
ros::Subscriber sub_handJoint;
ros::Subscriber sub_G_On_object[2];
ros::Subscriber sub_traget_of_robots[2];
ros::Subscriber sub_allocation_robots;
ros::Subscriber sub_end_pos[2];
ros::Subscriber sub_c_target;
ros::Publisher pub_handJoint;
ros::Publisher chatter_pub_object_left;
ros::Publisher chatter_pub_object_right;
ros::Publisher pub_handState;
ros::Publisher pub_gripper;

std_msgs::Int64 msg_hand_state;


CDDynamics *mFingerDyn;

sensor_msgs::JointState curr_joint_state;


ros::Time tstart;
ros::Time tend;
double jntcmd[4][4];
double velcmd[4][4];

bool gIsPlannerINIT = false;
sensor_msgs::JointState msgJoint;

double service_data_open[]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 88.0, 45.0, 0.0, 00.0};

double service_data_close[]={0.0, 90.0, 70.0, 30.0, 0.0, 90.0, 70.0, 30.0, 0.0, 90.0, 70.0, 30.0, 90.0, 45.0, 0.0, 70.0};


double allocation;

MathLib::Vector pos_t_a(16), vel_t_a(16), target_t_a(16);

MathLib::Vector Object_pos[2];
MathLib::Vector Robot_pos[2];
bool Position_of_the_object_recieved[2];
bool Position_of_the_end_recieved[2];
double time_initial;

MathLib::Vector	Pfirst_primitive[2];

int Target_selector[2];

bool new_select[2];
