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
	des_obj = -1;

	value = -1;
	force = 0;
	n_grippers = 0;

}

/*
Coalition::~Coalition()
{

}
*/

int Coalition::get_object_id()
{
	return des_obj;
}

std::vector<int>  Coalition::get_robots_id()
{
	std::vector<int> ids;
	for(const auto& rob : Robots)
	{
		ids.push_back(rob->get_id());
	}
	return ids;
}
void Coalition::assign()
{
	for(const auto& rob : Robots)
	{
		rob->set_assignment(des_obj);
	}
	Objects[des_obj]->set_assigned();
}

void Coalition::print_pointer()
{
	cout << this;
}


void Coalition::set_id(int id_)
{
	id = id_;
}

int Coalition::get_id()
{
	return id;
}

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

double Coalition::get_value()
{
	return value;
}

double Coalition::get_cost()
{
	return cost;
}

double Coalition::get_weight()
{
	return weight;
}

// TODO
// this should compute coal value, cost and weight. It should also set the task it wants to do (the one leading to lowest weight).
double Coalition::compute_value()
{
/*	for(auto& obj : Objects)
	{
		// check if the object can be done by this coalition
		if(is_feasible(*obj))
		{

		}
		else
			coalitional_value = 0;
	}*/

	if(value != 0)
		cost = 1/value;

	weight = value/n_robots;
	return value;
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
	cout << "coalitional value " << o.value << endl;
	cout << "task I want to do " << endl ;
/*	if(o.Assigned_obj != NULL)
		cout << o.Assigned_obj->get_id();
	else
		cout << "none, I'm a lazy POS";
	cout << endl;*/
}

