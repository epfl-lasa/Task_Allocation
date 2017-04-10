/*
  Copyright (c) 2016 Sina Mirrazavi,
  LASA Lab, EPFL, CH-1015 Lausanne, Switzerland,
  http://lasa.epfl.ch

  The program is free for non-commercial academic use.
  Please acknowledge the authors in any academic publications that have
  made use of this code or part of it.
  }
 */
#include "ball.h"




Command COM;
/*ENUM_State State;*/
double Postion_VO[3];


const int n_obj = 4; // patrick.


void chatterCallback_Command(const std_msgs::Int64& msg)
{
	int command;

	command=msg.data;

	switch(command){
	case COMMAND_INITIAL:
		COM=Com_Ball_INIT;
		break;
	case COMMAND_JOB:
		COM=Com_Ball_INIT;
		break;
	case COMMAND_Grab:
		COM=Com_Ball_Move;
		break;
	}
}
double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}



void chatterCallback_object(const visualization_msgs::InteractiveMarkerFeedback& msg)
{

	P_O(0)=msg.pose.position.x;
	P_O(1)=msg.pose.position.y;
	P_O(2)=msg.pose.position.z;
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub;
	ros::Publisher chatter_pub_p0; // pat
	ros::Publisher chatter_pub_p1; // pat
	ros::Publisher chatter_pub_p2;// pat
	ros::Publisher chatter_pub_p3;// pat
	ros::Publisher chatter_pub_f;
	ros::Publisher chatter_pub_acc_f;
	ros::Publisher chatter_pub_vel_f;
	ros::Publisher chatter_pub_object_left;
	ros::Publisher chatter_pub_object_right;
	ros::Subscriber sub_command;
	ros::Subscriber sub_state;
	ros::Subscriber object_state;
	ros::Subscriber object_state_p1; // pat
	ros::Subscriber object_state_p2;// pat
	ros::Subscriber object_state_p3;// pat
	geometry_msgs::Pose Object;
	geometry_msgs::Pose Object_p0; // pat
	geometry_msgs::Pose Object_p1;// pat
	geometry_msgs::Pose Object_p2;// pat
	geometry_msgs::Pose Object_p3;// pat
	geometry_msgs::Pose Object_f;
	geometry_msgs::Pose Object_vel;
	geometry_msgs::Pose Object_acc;
	geometry_msgs::Pose Object_left;
	geometry_msgs::Pose Object_right;

	std::vector<geometry_msgs::Pose> obj_pos;
	std::vector<geometry_msgs::Pose> obj_vel;
	std::vector<geometry_msgs::Pose> obj_acc;


	std::vector<ros::Publisher> pub_obj_pos; // pat
	std::vector<ros::Publisher> pub_obj_vel; // pat
	std::vector<ros::Publisher> pub_obj_acc; // pat




	chatter_pub = n.advertise<geometry_msgs::Pose>("/object/raw/position", 3);
	chatter_pub_f = n.advertise<geometry_msgs::Pose>("/object/filtered/position", 3);
	chatter_pub_vel_f = n.advertise<geometry_msgs::Pose>("/object/filtered/velocity", 3);
	chatter_pub_acc_f = n.advertise<geometry_msgs::Pose>("/object/filtered/acceleration", 3);
	chatter_pub_object_left = n.advertise<geometry_msgs::Pose>("/object/filtered/left/position", 3);
	chatter_pub_object_right = n.advertise<geometry_msgs::Pose>("/object/filtered/right/position", 3);



	sub_command = n.subscribe("/command", 3, chatterCallback_Command);
	//sub_state = n.subscribe("/catch/state", 3, chatterCallback_State);
	object_state = n.subscribe("/target_interaction/feedback", 3, chatterCallback_object);

	// pat
	std::ostringstream oss;
	for(int i = 0; i < n_obj; i++)
	{
		obj_pos.push_back(geometry_msgs::Pose());
		obj_vel.push_back(geometry_msgs::Pose());
		obj_acc.push_back(geometry_msgs::Pose());

		oss.str("");
		oss.clear();
		oss << "/object/p" << i << "/position";
		pub_obj_pos.push_back(n.advertise<geometry_msgs::Pose>(oss.str(), 1));

		oss.str("");
		oss.clear();
		oss << "/object/p" << i << "/velocity";
		pub_obj_vel.push_back(n.advertise<geometry_msgs::Pose>(oss.str(), 1));

		oss.str("");
		oss.clear();
		oss << "/object/p" << i << "/acceleration";
		pub_obj_acc.push_back(n.advertise<geometry_msgs::Pose>(oss.str(), 1));
	}


	chatter_pub_p0 = n.advertise<geometry_msgs::Pose>("/object/p0/position", 3);
	chatter_pub_p1 = n.advertise<geometry_msgs::Pose>("/object/p1/position", 3);
	chatter_pub_p2 = n.advertise<geometry_msgs::Pose>("/object/p2/position", 3);
	chatter_pub_p3 = n.advertise<geometry_msgs::Pose>("/object/p3/position", 3);

	filter	 = new SGF::SavitzkyGolayFilter(dim,order, winlen, sample_time);

	double P_I[3];
	double DP_I[3];
	P_O.Resize(3);
	Shift_left_P_O.Resize(3);Shift_left_P_O.Zero();
	Shift_right_P_O.Resize(3);Shift_right_P_O.Zero();
	DP_O.Resize(3);
	DDP_O.Resize(3);
	dt=0.005;

    Shift_left_P_O(1)=-0.10;
    Shift_right_P_O(1)=0.10;

	P_I[0]=-3.5;P_I[1]=-0.45;P_I[2]=0.8;
	DP_I[0]=0.1;DP_I[1]=0.0;DP_I[2]=0.0; //DP_I[0]=1.2;DP_I[1]=0.0;DP_I[2]=1.1;

	P_O(0)=P_I[0];P_O(1)=P_I[1];P_O(2)=P_I[2];
	DP_O(0)=DP_I[0];DP_O(1)=DP_I[1];DP_O(2)=DP_I[2];
	DDP_O(0)=0.0;DDP_O(1)=0.0;DDP_O(2)=0.0;//-0.5

	Vector DDhandle;
	DDhandle.Resize(3);DDhandle.Zero();

	Vector Dhandle;
	Dhandle.Resize(3);

	Vector handle;
	handle.Resize(3);

	ros::Rate r(200);
	//State=Com_Safe;
	COM=Com_Ball_INIT;

	double AA=0.1;
	while ((ros::ok()))
	{
		if (COM==Com_Ball_Move)
		{
			/*			switch(State){
			case Com_Stop:
		//		P_O.Set(Postion_VO,3);
				break;
			case Com_Break:
				P_O.Set(Postion_VO,3);
				cout<<"Break"<<endl;
				break;
			case Com_Safe:*/
			/*DDP_O.Mult(dt,DDhandle);
			DP_O=DP_O+DDhandle;
			DP_O.Mult(dt,Dhandle);
			P_O=P_O+Dhandle;*/
			/*				break;
			}*/

			/*			Object.position.x=P_O(0)+fRand(-0.05*AA,0.05*AA);
			Object.position.y=P_O(1)+fRand(-0.05*AA,0.05*AA);
			Object.position.z=P_O(2)+fRand(-0.05*AA,0.05*AA);*/

			Object.position.x=P_O(0);
			Object.position.y=P_O(1);
			Object.position.z=P_O(2);
			Object.orientation.x=0;
			Object.orientation.y=0;
			Object.orientation.z=0;
			Object.orientation.w=1;

			Object_left.position.x=P_O(0)+Shift_left_P_O(0);
			Object_left.position.y=P_O(1)+Shift_left_P_O(1);
			Object_left.position.z=P_O(2)+Shift_left_P_O(2);
			Object_left.orientation.x=0;
			Object_left.orientation.y=0;
			Object_left.orientation.z=0;
			Object_left.orientation.w=1;

			Object_right.position.x=P_O(0)+Shift_right_P_O(0);
			Object_right.position.y=P_O(1)+Shift_right_P_O(1);
			Object_right.position.z=P_O(2)+Shift_right_P_O(2);
			Object_right.orientation.x=0;
			Object_right.orientation.y=0;
			Object_right.orientation.z=0;
			Object_right.orientation.w=1;
		}
		if (COM==Com_Ball_INIT)
		{/*
			P_O(0)=P_I[0]+fRand(-AA,AA);P_O(1)=P_I[1]+fRand(-AA,AA);P_O(2)=P_I[2]+fRand(-AA,AA);
			DP_O(0)=DP_I[0]+fRand(-AA,AA);DP_O(1)=DP_I[1]+fRand(-AA,AA);DP_O(2)=DP_I[2];*/
		/*	P_O(0)=P_I[0]+fRand(-AA,AA);P_O(1)=P_I[1]+fRand(-AA,AA);P_O(2)=P_I[2]+fRand(-AA,AA);
				P_O(0)=P_I[0];P_O(1)=P_I[1];P_O(2)=P_I[2];
			DP_O(0)=DP_I[0];DP_O(1)=DP_I[1];DP_O(2)=DP_I[2];*/



			Object.position.x=P_O(0);
			Object.position.y=P_O(1);
			Object.position.z=P_O(2);

			/*			Object.position.x=P_O(0)+fRand(-0.05*AA,0.05*AA);
			Object.position.y=P_O(1)+fRand(-0.05*AA,0.05*AA);
			Object.position.z=P_O(2)+fRand(-0.05*AA,0.05*AA);*/

			Object.orientation.x=0;
			Object.orientation.y=0;
			Object.orientation.z=0;
			Object.orientation.w=1;

			Object_left.position.x=P_O(0)+Shift_left_P_O(0);
			Object_left.position.y=P_O(1)+Shift_left_P_O(1);
			Object_left.position.z=P_O(2)+Shift_left_P_O(2);

			/*			Object_left.position.x=P_O(0)+Shift_left_P_O(0)+fRand(-0.05*AA,0.05*AA);
			Object_left.position.y=P_O(1)+Shift_left_P_O(1)+fRand(-0.05*AA,0.05*AA);
			Object_left.position.z=P_O(2)+Shift_left_P_O(2)+fRand(-0.05*AA,0.05*AA);*/


			Object_left.orientation.x=0;
			Object_left.orientation.y=0;
			Object_left.orientation.z=0;
			Object_left.orientation.w=1;

			Object_right.position.x=P_O(0)+Shift_right_P_O(0);
			Object_right.position.y=P_O(1)+Shift_right_P_O(1);
			Object_right.position.z=P_O(2)+Shift_right_P_O(2);

			/*Object_right.position.x=P_O(0)+Shift_right_P_O(0)+fRand(-0.05*AA,0.05*AA);
			Object_right.position.y=P_O(1)+Shift_right_P_O(1)+fRand(-0.05*AA,0.05*AA);
			Object_right.position.z=P_O(2)+Shift_right_P_O(2)+fRand(-0.05*AA,0.05*AA);*/

			Object_right.orientation.x=0;
			Object_right.orientation.y=0;
			Object_right.orientation.z=0;
			Object_right.orientation.w=1;

		//	State=Com_Safe;

		}


		inp(0) = Object.position.x;
		inp(1) = Object.position.y;
		inp(2) = Object.position.z;
		ret_code = filter->AddData(inp);
		ret_code = filter->GetOutput(0, outp);
		Object_f.position.x=outp(0);						Object_f.position.y=outp(1);						Object_f.position.z=outp(2);
		Object_f.orientation.x=Object.orientation.x;		Object_f.orientation.y=Object.orientation.y;		Object_f.orientation.z=Object.orientation.z;
		Object_f.orientation.w=Object.orientation.w;
		ret_code = filter->GetOutput(1, outp);
		Object_vel.position.x=outp(0);				Object_vel.position.y=outp(1);				Object_vel.position.z=outp(2);
		Object_vel.orientation.x=0;					Object_vel.orientation.y=0;					Object_vel.orientation.z=0;
		Object_vel.orientation.w=0;
		ret_code = filter->GetOutput(2, outp);
		Object_acc.position.x=outp(0);				Object_acc.position.y=outp(1);				Object_acc.position.z=outp(2);
		Object_acc.orientation.x=0;					Object_acc.orientation.y=0;					Object_acc.orientation.z=0;
		Object_acc.orientation.w=0;

		chatter_pub.publish(Object);
		chatter_pub_f.publish(Object_f);
		chatter_pub_vel_f.publish(Object_vel);
		chatter_pub_acc_f.publish(Object_acc);
		chatter_pub_object_left.publish(Object_left);
		chatter_pub_object_right.publish(Object_right);


		// patrick  make our objects here... hard-coded 2 in front, big in middle, one in back

		obj_pos[0] = Object;
	//	Object_p0 = Object;


		obj_pos[1].position.x = Object.position.x + 0.5;
		obj_pos[1].position.y = Object.position.y + 0.2;
		obj_pos[1].position.z = Object.position.z;

		obj_pos[2].position.x = Object.position.x + 0.5;
		obj_pos[2].position.y = Object.position.y - 0.2;
		obj_pos[2].position.z = Object.position.z;

		obj_pos[3].position.x = Object.position.x - 0.5;
		obj_pos[3].position.y = Object.position.y - 0.0;
		obj_pos[3].position.z = Object.position.z;


		// all the same atm, but writing like this lets us change them...
		obj_vel[0] = Object_vel;
		obj_vel[1] = Object_vel;
		obj_vel[2] = Object_vel;
		obj_vel[3] = Object_vel;

		obj_acc[0] = Object_acc;
		obj_acc[1] = Object_acc;
		obj_acc[2] = Object_acc;
		obj_acc[3] = Object_acc;

		for(int i = 0; i < n_obj; i++)
		{
			pub_obj_pos[i].publish(obj_pos[i]);
			pub_obj_vel[i].publish(obj_vel[i]);
			pub_obj_acc[i].publish(obj_acc[i]);
		}
	/*	chatter_pub_p0.publish(Object_p0);
		chatter_pub_p1.publish(Object_p1);
		chatter_pub_p2.publish(Object_p2);
		chatter_pub_p3.publish(Object_p3);
*/


		r.sleep();
		ros::spinOnce();
	}
	return 0;
}
