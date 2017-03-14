/*
 * Coalition.cpp
 *
 *  Created on: 12 mars 2017
 *      Author: Patrick
 */

#include "Coalition.h"


Coalition::Coalition()
{
	n_robots = 0;
	Robots = NULL;
	n_tasks = 0;
	Tasks = NULL;

	coalitional_value = -1;
	force = -1;
	n_grippers = -1;
}

Coalition::Coalition(int n_bots, Robot_agent** Robots_, int n_tasks_, Object** Tasks_)
{
	force = 0;
	n_grippers = 0;
	coalitional_value = 0;


	if(n_bots < 1)
	{
		cout << "Error in coalition constructor, n_robots below 1, value of n: " << n_bots << endl;
	}
	n_robots = n_bots;

	if(n_tasks_ < 1)
	{
		cout << "Error in coalition constructor, n_tasks below 1, value of n: " << n_bots << endl;
	}
	n_tasks = n_tasks_;
	Robots = new Robot_agent*[n_robots];
	for(int i = 0; i < n_robots; i++)
	{
		Robots[i] = Robots_[i];
	}

	Tasks = new Object*[n_tasks];
	for(int i = 0; i < n_tasks; i++)
	{
		Tasks[i] = Tasks_[i];
	}

	// compute force and number of grippers.
	for(int i = 0; i < n_robots; i++)
	{
		n_grippers += Robots[i]->get_n_grippers();
		force += Robots[i]->get_force();
	}
}


Coalition::~Coalition()
{
	if(Robots != NULL)
		delete Robots;

	if(Tasks != NULL)
		delete Tasks;
}

int Coalition::add_robot(Robot_agent* bot)
{
	// todo if needed
	return 0;

}

int Coalition::add_task(Object* Task)
{
	// todo if needed
	return 0;
}

int Coalition::get_n_robots()
{
	return n_robots;
}

int Coalition::get_n_grippers()
{
	return n_grippers;
}

double Coalition::get_force()
{
	return force;
}

double Coalition::get_coalitional_value()
{
	return coalitional_value;
}

double Coalition::evaluate_task(Object task)
{

	return 0;
}

double Coalition::compute_coalitional_value()
{



	return coalitional_value;
}
