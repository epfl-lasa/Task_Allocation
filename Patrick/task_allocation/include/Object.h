/* TODO
 *
 * add the functions from multiarm_ds that handle objects in here
 *
 * When something is modified in this file (eg a variable added), remember to update the copy constructor.
 * Copy constructor seems required because of the VectorXd[] things
 */



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
#include <string>
enum class Object_prediction_type{Ballistic,Straight};


class Object {

public:

	Object();
	Object(int N_state, VectorXd X, VectorXd DX, double max_time, double dt, VectorXd grabbing_states[], int n_grabbing_states, double weight, double value, int id_, Object_prediction_type Object_motion=Object_prediction_type::Straight );

//	Object( const Object &o); // copy construct

	void predict_motion();
	void dumb_predict_motion();

	// setters
	void set_prediction_parameters(double max_time_, double dt);
	void set_state(VectorXd X, VectorXd DX);
	void set_prediction_state(VectorXd X,VectorXd X_filtered, double time);
	void set_assignment(bool val);

	// getters
	VectorXd get_X_O() const;
	VectorXd get_DX_O() const;
	int get_id() const;
	double get_dt() const;
	double get_max_time() const;

	MatrixXd get_P_O_prediction() const;
	MatrixXd get_P_O_G_prediction(int index) const;
	double get_value() const;
	double get_weight() const;
	int	get_n_grippers() const;
	bool get_assignment() const;

	// various
	void print_estimator() const;
	double update_value();
	friend std::ostream& operator<< (std::ostream& stream, const Object& Object);

private:

	const static int max_grabbing_state = 4;

	const double minPos[3] = {-0.5, -1.00, 0.3};
	const double maxPos[3] = {0.0,  1.00, 1.0};

	int id;

	bool is_assigned;

	int n_state;


	int n_grabbing_pos; 				 // Number of the grabbing positions
	VectorXd X_O_First; 				// The State of the object with respect to the world frame
	VectorXd X_O;		 				// The State of the object with respect to the world frame

	VectorXd DX_O;						// The D-State of the object with respect to the world frame
	VectorXd X_O_G[max_grabbing_state];// The State of the grabbing positions with respect to the state of the object

	Object_prediction_type motion_type;
	double	max_pred_time;
	double dt;
	int n_frames;
	MatrixXd	P_O_prediction;
	MatrixXd	P_O_G_prediction[max_grabbing_state];


	double 		weight; // weight of the object, aka force required to lift it
	double		value; // value of the object


	TrajectoryEstimation *predict;

};


inline void printVector(string name, VectorXd vec)
{
	cout << name << endl << vec << endl;
}


#endif
