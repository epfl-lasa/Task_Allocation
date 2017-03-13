#ifndef OBJECT_H
#define OBJECT_H

#include <stdio.h>
#include <stdlib.h>
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "LPV.h"
#include "TrajectoryEstimation.h"
#include <math.h>
#include  <omp.h>

enum ENUM_State_of_prediction{Ballistic,Straight};

const int Max_Grabbing_state = 4;

const double dt = 0.1; // this is used for the trajectory estimation, not too sure what to do with it...

class Object {

public:

	Object();
	Object(int N_state, VectorXd X, VectorXd DX, double max_time, VectorXd grabbing_states[], int n_grabbing_states, double weight, double value, ENUM_State_of_prediction Object_motion );

	void predict_motion();

	// setters
	void set_max_pred_time(double time);
	void set_state(VectorXd X, VectorXd DX);


	// getters
	bool get_state_set();
	bool get_first_state_set();
	bool get_grabbing_state_set(int i);
	double get_value();
	double get_weight();

private:

	int n_state;

	bool state_is_set;
	bool first_state_is_set;
	bool grabbing_state_is_set[Max_Grabbing_state];
	ENUM_State_of_prediction motion_type;
	double	max_pred_time; // patrick changed, remove const
	int n_grabbing_pos; 				 // Number of the grabbing positions
	VectorXd X_O_First; 				// The State of the object with respect to the world frame
	VectorXd X_O;		 				// The State of the object with respect to the world frame
	VectorXd X_O_INTERCEPT;			// The State of the object with respect to the intercept point
	VectorXd X_I_C;		 			// The State of the desired intercept point with respect to the world frame
	VectorXd DX_O;						// The D-State of the object with respect to the world frame
	VectorXd X_O_G[Max_Grabbing_state];// The State of the grabbing positions with respect to the state of the object
	TrajectoryEstimation *predict;

	int n_frames;
	MatrixXd	P_O_prediction;
	MatrixXd	P_O_G_prediction[Max_Grabbing_state];
	MatrixXd	order_of_grabbing;
	MatrixXd	prob_order_of_grabbing;
	double		max_liklihood; // prob useless for me
	int 		index_row; // prob useless for me
	int 		index_column; // prob useless for me

	double 		weight; // weight of the object, aka force required to lift it
	double		value; // value of the object


};




#endif
