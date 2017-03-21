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
//	Robots.reserve(max_n_robots);

	n_objects = 0;
//	Objects.reserve(max_n_objects);
//	obj_values.reserve(max_n_objects);
	assigned_obj = -1;

	coalitional_value = -1;
	force = 0;
	n_grippers = 0;

}

/*
Coalition::~Coalition()
{

}
*/

int Coalition::add_robot(Robot_agent* bot)
{
	// todo if needed
	if(n_robots < max_n_robots)
	{
		n_robots++;
		Robots.push_back(bot);
		force += bot->get_force();
		n_grippers += bot->get_n_grippers();
		return n_robots;
	}
	return -1;

}

int Coalition::add_task(Object* Obj)
{
	// todo if needed
	if(n_objects < max_n_objects)
	{
		n_objects++;
		Objects.push_back(Obj);
		return n_objects;
	}
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

double Coalition::compute_coalitional_value()
{
	for(auto& obj : Objects)
	{
		// check if the object can be done by this coalition
		if(is_feasible(*obj))
		{

		}
		else
			coalitional_value = 0;
	}
	return coalitional_value;
}

bool Coalition::is_feasible(Object& obj)
{
	bool feasible = false;

	if(n_grippers >= obj.get_n_grippers())
		if(force >= obj.get_weight())
			feasible = true;

	return feasible;
}


std::ostream& operator <<(std::ostream& stream, const Coalition& o)
{
	cout << "n robots " << o.n_robots << endl;
	cout << "Robot IDs in this coalition ";
	for(int i = 0; i < o.n_robots; i++)
		cout << o.Robots[i]->get_id() << " " ;
	cout << endl;

	cout << "n grippers " << o.n_grippers << endl;
	cout << "force " << o.force << endl;
	cout << "objects available " << o.n_objects << endl;
	cout << "coalitional value " << o.coalitional_value << endl;
	cout << "task I want to do " << endl ;
/*	if(o.Assigned_obj != NULL)
		cout << o.Assigned_obj->get_id();
	else
		cout << "none, I'm a lazy POS";
	cout << endl;*/
}

