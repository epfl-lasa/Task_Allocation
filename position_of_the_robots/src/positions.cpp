#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

double Posiiton_7[3]={0,0,0};
double Posiiton_14[3]={0,0,0};


void position_14(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	/*	X_new=Y_vision
		Z_new=Z_vision
		Y_new=-X_vision
	 */
	Posiiton_14[0]=msg->pose.position.y;
	Posiiton_14[1]=-msg->pose.position.x;
	Posiiton_14[2]=msg->pose.position.z;

	if (Posiiton_7[0]!=0)
	{
		std::cout<<"The position of KUKA 14 is  "<<Posiiton_14[0]-Posiiton_7[0]<<" "<<Posiiton_14[1]-Posiiton_7[1]<<" "<<Posiiton_14[2]-Posiiton_7[2]<<std::endl;
	}
}

void position_7(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	/*	X_new=Y_vision
	Z_new=Z_vision
	Y_new=-X_vision
	 */
	Posiiton_7[0]=msg->pose.position.y;
	Posiiton_7[1]=-msg->pose.position.x;
	Posiiton_7[2]=msg->pose.position.z;

}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "listener");

	ros::NodeHandle n;
	ros::Subscriber sub_14 = n.subscribe("/KUKA_14/pose", 1000, position_14);
	ros::Subscriber sub_7 = n.subscribe("/KUKA_7/pose", 1000, position_7);

	ros::spin();
	return 0;

}
