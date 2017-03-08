#include "sg_filter.h"
#include "math.h"
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"



int order = 5;
int winlen = 50;
SGF::real sample_time = 0.005;
int dim=3;
SGF::Vec inp(3);
SGF::Vec outp(3);
int ret_code;
ros::Publisher chatter_pub;
ros::Publisher chatter_pub_vel;
ros::Publisher chatter_pub_acc;
geometry_msgs::Pose Object;
geometry_msgs::Pose Object_vel;
geometry_msgs::Pose Object_acc;


SGF::SavitzkyGolayFilter *filter;


void chatterCallback_object(const geometry_msgs::Pose & msg)
{

	inp(0) = msg.position.x;
	inp(1) = msg.position.y;
	inp(2) = msg.position.z;
	ret_code = filter->AddData(inp);
	//if (ret_code==-2) {std::cout<<"The dimension of the data is wrong!"<<std::endl;}
	ret_code = filter->GetOutput(0, outp);
	if (ret_code==0)
	{
		Object.position.x=outp(0);					Object.position.y=outp(1);					Object.position.z=outp(2);
		Object.orientation.x=msg.orientation.x;		Object.orientation.y=msg.orientation.y;		Object.orientation.z=msg.orientation.z;
		Object.orientation.w=msg.orientation.w;
		ret_code = filter->GetOutput(1, outp);
		Object_vel.position.x=outp(0);				Object_vel.position.y=outp(1);				Object_vel.position.z=outp(2);
		Object_vel.orientation.x=0;					Object_vel.orientation.y=0;					Object_vel.orientation.z=0;
		Object_vel.orientation.w=0;
		ret_code = filter->GetOutput(2, outp);
		Object_acc.position.x=outp(0);				Object_acc.position.y=outp(1);				Object_acc.position.z=outp(2);
		Object_acc.orientation.x=0;					Object_acc.orientation.y=0;					Object_acc.orientation.z=0;
		Object_acc.orientation.w=0;
		chatter_pub.publish(Object);
		chatter_pub_vel.publish(Object_vel);
		chatter_pub_acc.publish(Object_acc);
	}


}



int main(int argc, char **argv) {

	ros::init(argc, argv, "filter");
	ros::NodeHandle n;

	ros::Subscriber	sub_object = n.subscribe("/object/raw/position", 3, chatterCallback_object);
	chatter_pub = n.advertise<geometry_msgs::Pose>("/object/filtered/position", 3);
	chatter_pub_vel = n.advertise<geometry_msgs::Pose>("/object/filtered/velocity", 3);
	chatter_pub_acc = n.advertise<geometry_msgs::Pose>("/object/filtered/acceleration", 3);


	filter= new SGF::SavitzkyGolayFilter(dim,order, winlen, sample_time);
	ros::spin();

	return 0;
}
