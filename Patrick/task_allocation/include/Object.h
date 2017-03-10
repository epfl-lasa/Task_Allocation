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



class Object {

public:

	Object();
	Object(int N_state, VectorXd X, VectorXd DX, double max_time, MatrixXd grabbing_states, int n_grabbing_states );

	predict_motion();
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

	int N_state_;
	bool State_is_set_;
	bool First_state_is_set_;
	bool Grabbing_state_is_set_[Max_Grabbing_state];
	ENUM_State_of_prediction Object_motion_;
	double	max_pred_time; // patrick changed, remove const
	int N_grabbing_pos_; 				 // Number of the grabbing positions
	VectorXd X_O_First_; 				// The State of the object with respect to the world frame
	VectorXd X_O_;		 				// The State of the object with respect to the world frame
	VectorXd X_O_INTERCEPT_;			// The State of the object with respect to the intercept point
	VectorXd X_I_C_;		 			// The State of the desired intercept point with respect to the world frame
	VectorXd DX_O_;						// The D-State of the object with respect to the world frame
	VectorXd X_O_G_[Max_Grabbing_state];// The State of the grabbing positions with respect to the state of the object
	TrajectoryEstimation *predict_;
	MatrixXd	P_O_prediction_;
	MatrixXd	P_O_G_prediction_[Max_Grabbing_state];
	MatrixXd	order_of_grabbing_;
	MatrixXd	prob_order_of_grabbing_;
	double		Max_liklihood;
	int 		index_row;
	int 		index_column;

	double 		weight; // weight of the object, aka force required to lift it
	double		value; // value of the object

//	S_Coalition coalition; // coalition that is assigned to this object

};




#endif
