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
	des_obj = nullptr;

	value = 0;
	weight = 0;
	cost = 0;
	force = 0;
	n_grippers = 0;

}

/*
Coalition::~Coalition()
{

}
*/

int Coalition::get_object_id() const
{
	if(des_obj != nullptr)
		return des_obj->get_id();
	return -1;
}

std::vector<int>  Coalition::get_robots_id() const
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
	// assign all robots in here
	for(const auto& rob : Robots)
	{
    //    cout << "setting robot " << rob->get_id() << " assigned to " << des_obj->get_id() << endl;
		rob->set_assignment(des_obj->get_id());
    //    rob->set_status(Robot_status::Allocated);
	}

    des_obj->set_assigned();
}

void Coalition::print_pointer() const
{
	cout << this;
}


void Coalition::set_id(int id_)
{
	id = id_;
}

int Coalition::get_id() const
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

int Coalition::get_n_robots() const
{
	return n_robots;
}

int Coalition::get_n_grippers() const
{
	return n_grippers;
}

double Coalition::get_force() const
{
	return force;
}

double Coalition::get_value() const
{
	return value;
}

double Coalition::get_cost() const
{
	return cost;
}

double Coalition::get_weight() const
{
	return weight;
}

// TODO
// this should compute coal value, cost and weight. It should also set the task it wants to do (the one leading to lowest weight).
double Coalition::compute_value()
{

	weight = 100000;
	value = 1000000;
	cost = 1000000;
	double temp_weight;
	double temp_value = 100000;
	double temp_cost = 0;
	// check for each object
	for(const auto& obj : Objects)
	{
		if(obj->is_done() == false)
		{
			// check if the object can be done by this coalition
			if(is_feasible(*obj))
			{
				temp_cost = 0;
	//			cout << "computing cost ";
				for(const auto& rob : Robots)
				{
					temp_cost += rob->evaluate_task(*obj);
		//			cout << temp_cost << " " ;
				}
			//	cout << endl;
	//			cout << "evaluated cost " << temp_cost;
				temp_value = obj->get_value();
		//		cout << " evaluated value " << temp_value;
                temp_weight = temp_value - temp_cost;
				temp_weight /= n_robots;
				temp_weight = 1/(temp_weight);

     /*           if((Robots[0]->get_id() == 2 && Robots[1]->get_id() == 3) || (Robots[0]->get_id() == 3 && Robots[0]->get_id() == 2))
                {
                    if(obj->get_id() == 0)
                        cout << "temp_cost " << temp_cost << " temp weight " << temp_weight << endl;
                }
		/*		if(n_robots == 1) // robot 1 is at (0,0,0)
				{
					if(temp_weight > 0)
					{
						if(Robots[0]->get_id() == 1)
							cout << " cost " << temp_cost << " for object " << obj->get_id();
					}
				}//   */

     //           cout << "robot " << Robots[0]->get_id() << " object " << obj->get_id() << " value " << obj->get_value() << " cost " << temp_cost << endl;



                if(0.0 < temp_weight && temp_weight < weight)
				{
		//			cout << " updated the desired object" << endl;
					value = temp_value;
					weight = temp_weight;
					cost = 1/value;
					des_obj = obj;
				}
			}
	//		cout << "computed value, the object I want is " << obj->get_id() << endl;
	//		if(weight < 5)
		//		weight = -1;
		}
	}

//	cout << endl;
//	cout << "weight " << weight << " id " << id << " object " << des_obj->get_id() << endl;

	return value;
}


/*   // debug stuff
void Coalition::print_intercept() const
{
	Robots[0]->compute_intercept(*des_obj);
	cout << "intercept for robot " << Robots[0]->get_id() << " is " << endl <<  Robots[0]->get_intercept() << endl;
}
*/
// debug stuff
void Coalition::compute_intercept()
{
	Robots[0]->compute_intercept(*des_obj);
}


bool Coalition::is_feasible(Object& obj) const
{
	bool feasible = false;
    if(n_grippers == obj.get_n_grippers())
    {

     //   cout << "coalition of " << Robots.size() << " can do object " << obj.get_id() << " requiring " << obj.get_n_grippers() << " grippers " << endl;
        if(force >= obj.get_weight())
				feasible = true;
    }
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
	cout << "coalitional cost " << o.cost << endl;
	cout << "coalitional weight " << o.weight << endl;
	if(o.des_obj != nullptr)
		cout << "task I want to do " << (o.des_obj)->get_id();
	else
		cout << "task I want to do " << -1;
	cout << endl;
}

