/*
 * Coalition.h
 *
 *  Created on: 12 mars 2017
 *      Author: Patrick
 */

#ifndef COALITION_H_
#define COALITION_H_


class Coalition {

public:

	int get_n_robots();
	int get_n_grippers();
	double get_force();
	double get_coalitional_value();

	double evaluate_task(Object task);


private:
	int n_robots;   // number of robots
	int* robots_id; // IDs of the robots so we can adress them later

	int n_grippers; // available grippers
	double force; // available force

	double* values; // tasks values
	double coalitional_value;



};


#endif
