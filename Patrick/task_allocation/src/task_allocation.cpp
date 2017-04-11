/*
 * Copyright (C) 2016 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Sina Mirrazavi
 * email:   sina.mirrazavi@epfl.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#include "task_allocation.h"


Task_allocation::Task_allocation()
{
	cout << "you called an empty Task Allocation constructor and shouldn't have done that" << endl;
	n_objects = 0;
	n_robots = 0;
}

Task_allocation::Task_allocation(double max_time_, double dt_, int n_state_, Object_prediction_type Object_motion)
{

	n_robots = 0;
	n_objects = 0;
	n_coalitions = 0;
	Robots.clear();
	Objects.clear();
	active_coalitions.clear();
	Coalitions.clear();
//	Multi_ds = DS_;


	n_state = n_state_;

	max_pred_time = max_time_;
	dt = dt_;
	n_frames = floor(max_time_/dt)+1;
	Prediction_model = Object_motion;

}


/*
Robot_agent* Task_allocation::get_robot(int i)
{
	if(i >= 0 && i < Robots.size())
		{
			return &(Robots[i]);
		}
		cout << "wrong parameter given to task_allocation::get_object(i), I might segfault. i = " << i << endl;
		return &(Robots[0]); // just pray that there is one....
}
*/
/*
Object* Task_allocation::get_object(int i)
{
	if(i >= 0 && i < Objects.size())
	{
		return &(Objects[i]);
	}
	cout << "wrong parameter given to task_allocation::get_object(i), I might segfault. i = " << i << endl;
	return &(Objects[0]); // just pray that there is one....
//	return (Object*)nullptr;
}
*/

VectorXd Task_allocation::get_object_state(int i)
{
	VectorXd empty_vec;
	empty_vec.resize(n_state);
	empty_vec.setConstant(-1);
	if(i >= 0 && i < n_objects)
		return Objects[i]->get_X_O();
	else
		return empty_vec;
}


VectorXd Task_allocation::get_robot_intercept(int i) const
{
	VectorXd empty_vec;
	empty_vec.resize(n_state);
	empty_vec.setConstant(-1);
	if(i >= 0 && i < n_robots)
	{
		return Robots[i]->get_intercept();
	}
	return VectorXd();
}


void Task_allocation::predict_motion()
{

	for(auto & obj : Objects)
		obj->dumb_predict_motion();

}


void Task_allocation::clear_coalitions()
{
	Coalitions.clear();
	active_coalitions.clear();
	unallocated_robots.clear();
	for(auto& rob : Robots)
	{
		rob->set_assignment(-1);
	}


	for(auto& obj : Objects)
	{
		obj->set_assignment(false);
	}
}


void Task_allocation::compute_coordination()
{
	targets.resize(6, n_robots);
	targets.setZero();
	std::vector<int> robId;
	coordinations.clear();
	for(auto & coal : active_coalitions)
	{
		robId = coal.get_robots_id();
		if(coal.get_n_robots() == 1)
		{
		//	cout << "robot in this coalition " << robId[0] << endl;
			targets.col(robId[0]) = get_robot_intercept(robId[0]);
			coordinations.push_back(0);
	//		Multi_ds->Set_coordination(robId[0],0);
		//	Multi_ds->Set_the_robot_first_primitive_desired_position(robId[0], targets.col(robId[0]), zeroVec);
		}
		else
		{
			for(auto i : robId)
			{
				coordinations.push_back(1);
		//		Multi_ds->Set_coordination(i,1);
			}
		}
	}

	for(auto & rob : Robots)
	{
		if(rob->get_assignment() == -1)
			targets.col(rob->get_id()) = rob->get_intercept();
	}
}

MatrixXd Task_allocation::get_targets()
{
	return targets;
}

std::vector<double> Task_allocation::get_coordinations()
{
	return coordinations;
}

// computes all coalitions for the unallocated robots.
void Task_allocation::build_coalitions()
{

//	Coalitions.reserve(MAX_COALITION_SIZE);
	Coalitions.clear();
	unallocated_robots.clear();

//cout << "build_coalitions: building unallocated_robots ...";
// ****** look for unallocated robots
	for(auto& rob : Robots)
	{
		if(rob->get_assignment() ==  -1)
		{
			unallocated_robots.push_back(rob);
		}
	}
//cout << "... done" << endl;

// ****** make all the possible coalitions
//cout << "build_coalitions: making the coalitions ..." << endl;
	int n_bots = unallocated_robots.size();
	for(int i = 0; i < min(MAX_COALITION_SIZE, n_bots); i++)
	{

		// ************* make the matrix with all permutations of the available robots
		unsigned long int number_of_coalitions;
		MatrixXd perm = PermGenerator(n_bots,i+1); // i+1 because in array 0 we store the coalitions of size 1(ie singletons)

		int n_rows = perm.rows();
		int n_cols = perm.cols();
		bool dupe = false;
		int to_remove[n_rows];
		int n_dupes = 0;
		for(int j = 0; j < n_rows; j++)
			to_remove[j] = -1;

		// remove the duplicates, as the order does not matter in our case
//		cout << "perm matrix for i = " << i << " and n_bots = " << n_bots << endl << perm << endl;
		for(int u = 0; u < perm.rows() - 1; u++)
		{

			for(int j = u+1; j < perm.rows(); (dupe == false) ? j++ : j) // only increment if the previous wasn't a dupe, if it's a dupe, as we remove it, we need to check the same row
			{
				dupe = check_dupe(perm.row(u), perm.row(j));

				if(dupe == true)
				{
//					cout << "removing row " << perm.row(j) << " as it is similar to " << perm.row(u) << endl;
					removeRow(perm, j);

				}
			}
		}


	//	cout << "perm matrix for i = " << i << " and n_bots = " << n_bots << " without dupes " << endl << perm << endl;

		number_of_coalitions = perm.rows();

		Coalitions.push_back( std::vector<Coalition>() );
//		Coalitions[i].reserve(number_of_coalitions);

		// the 2nd level is the ID of the coalition within that size
		for(int j = 0; j < number_of_coalitions; j++)
		{

			// make the coalition
			Coalition test;
			for(int k = 0; k < i+1; k++) // add all robots that should be in this coalition.
			{
				test.add_robot((unallocated_robots[perm(j,k)]));
			}
			for(int k = 0; k < n_objects; k++)
			{
				if(Objects[k]->get_assignment() == false)
					test.add_task((Objects[k]));
			}
			test.set_id(i*min(MAX_COALITION_SIZE, n_bots)+j);
			Coalitions[i].push_back(test);
//			cout << "Coalition of size " << i+1 << " number " << j << endl << Coalitions[i][j] << endl;
		}
	}


	// some tests on pointers and addresses and stuff in this part. This is just to confirm something
/*	double lowest_weight = 100000;
	double temp_weight = lowest_weight;
	Coalition* low_coal = NULL;
	int i = 0;
	int j = 0;
	for(auto& row : Coalitions)
	{

		for(auto& coal : row)
		{

			temp_weight = coal.get_weight();
			if(temp_weight < lowest_weight)
			{
				lowest_weight = temp_weight;
				low_coal = &(coal);
				cout << "comparing the 2 pointers " << low_coal << " " ;
				coal.print_pointer();
				cout << " and accessing with i j we get ";
				((Coalitions.at(i)).at(j)).print_pointer();
			}
			j++;
		}
		i++;
	}
*/
}



void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}

int Task_allocation::add_robot(Robot_agent* bot)
{
	Robots.push_back(bot);
	n_robots++;

	return n_robots;
}


int Task_allocation::add_task(Object* task)
{
	Objects.push_back(task);
	n_objects++;

	return n_objects;
}

bool Task_allocation::set_object_state(int i, VectorXd X, VectorXd DX)
{
	if(i < n_objects)
	{
		Objects[i]->set_state(X, DX);

		return true;
	}
	return false;
}

void Task_allocation::print_obj()  const
{
	for(const auto& obj : Objects)
		obj->print_estimator();
}


void Task_allocation::allocate()
{
	// **************** reset everything

	// we are beginning, there's no active coalition, all robots are unallocated


	clear_coalitions();

	for(int i = 0; i < n_objects; i++) // the boundary should be something else....
	{
		// *************** build the coalitions
	//	cout << "building coalitions" << endl;
		build_coalitions();

		int n_coal = 0;
		for(auto& row: Coalitions)
			for(auto& coal : row)
				n_coal++;
//		cout << "done building coalitions: " << unallocated_robots.size() << " available robots, making " << n_coal << " coalitions " << endl;
		if(unallocated_robots.size() < 1)
		{
			break;
		}


		// ************** evaluate the coalitions
	//	cout << "evaluating coalitions" << endl;
	//	evaluate_coalitions();

		double lowest_weight = 100000;
		double temp_weight = lowest_weight;
		Coalition* low_coal = nullptr;
		for(auto& row : Coalitions)
		{
			for(auto& coal : row)
			{
				coal.compute_value();
				temp_weight = coal.get_weight();
				if(0.0 < temp_weight && temp_weight < lowest_weight)
				{
					lowest_weight = temp_weight;
					low_coal = &(coal);
				}
			}
		}


		// *******************
		// add the corresponding coalition to the active coalitions
		// set the assigned task in this coalition
		// set the assignment (target task) of the robots in this coalition
		if(low_coal != nullptr)
		{
	//		cout << "found best coalition" << endl;
	//		cout << *low_coal << endl;
			active_coalitions.push_back(*low_coal);
			low_coal->assign();
		}
	//	else
		//	cout << "didn't find a coalition" << endl;
	}



}

int Task_allocation::get_n_coals() const
{
	return active_coalitions.size();
}

int Task_allocation::get_robot_target(int i) const
{
	if(i >= 0 && i < Robots.size())
		return Robots[i]->get_assignment();
	else
		return -1;
}

void Task_allocation::print_bases() const
{
	for(const auto & rob : Robots)
	{
		cout << "Robot " << rob->get_id() << " base at " << rob->get_base() << endl;
	}
}

void Task_allocation::update_objects_value()
{
	for(auto & obj : Objects)
	{
		obj->update_value();
	}
}

void Task_allocation::print_intercepts() const
{
	for(const auto & coal : active_coalitions)
	{
		if(coal.get_n_robots() == 1)
		{
			coal.print_intercept();
		}
	}
}

void Task_allocation::compute_intercepts()
{
	for(auto & coal : active_coalitions)
	{
		if(coal.get_n_robots() == 1)
		{
			coal.compute_intercept();
		}
	}

	for(auto & rob : Robots)
	{
		if(rob->get_assignment() == -1) // robot is not assigned
		{
			rob->set_idle_target();
		}
	}
}



bool Task_allocation::set_robot_state(int i, VectorXd X_)
{
	if(i > 0 && i < n_robots)
	{
		Robots[i]->set_state(X_);
		return true;
	}
	return false;
}

void Task_allocation::multi_frame_allocation()
{

}


double Task_allocation::get_dt() const
{
	return dt;
}

double Task_allocation::get_max_time() const
{
	return max_pred_time;
}

int Task_allocation::factorial(int n)
{
    if(n > 1)
        return n * factorial(n - 1);
    else
        return 1;
}


MatrixXd Task_allocation::PermGenerator(int n, int k)
{
	MatrixXd handle(factorial(n)/factorial(n-k),k);
    std::vector<int> d(n);
    std::iota(d.begin(),d.end(),1);
//    cout << "These are the Possible Permutations: " << endl;
    int repeat = factorial(n-k);
    int counter=0;
    do
    {
        for (int i = 0; i < k; i++)
        {
        	handle(counter,i)=d[i]-1;
        }
        counter=counter+1;
        for (int i=1; i!=repeat; ++i)
        {
            next_permutation(d.begin(),d.end());
        }
    } while (next_permutation(d.begin(),d.end()));

  //  cout<<handle<<endl;

    return handle;
}



bool Task_allocation::check_dupe(const VectorXd& rowA, const VectorXd& rowB)
{
	int dupe = 0;
	bool found;

	if(rowA.rows() != rowB.rows())
		return false;

	for(int i = 0; i < rowA.rows(); i++)
	{
		for(int j = 0; j < rowB.rows(); j++)
		{
			if(rowA(i) == rowB(j))
			{
				found = true;
				break;
			}
		}

		if(found != true)
			return false;

	}
	return true;

}



void Task_allocation::print_coalitions() const
{
	if(active_coalitions.size() < 1)
	{
	//	cout << "no coalitions" << endl;
		return;
	}

	for(const auto& coal : active_coalitions)
	{
		cout << endl << "***** BEGIN OF COALITION  *****" << endl;
		cout << coal;
		cout << endl << "***** END OF COALITION  *****" << endl;
	}

}


std::ostream& operator<< (std::ostream& stream, const Task_allocation& o)
{
	cout << endl << "**************** BEGIN OF TASK ALLOCATION ****************" << endl;

	cout << "n state " << o.n_state << endl;

	cout << "n robots " << o.n_robots << endl;
	cout << "n objects " << o.n_objects << endl;


	for(const auto& rob : o.Robots)
	{
		cout << endl << "***** BEGIN OF ROBOT *****" << endl;
		cout << *rob;
		cout << "***** END OF ROBOT *****" << endl;
	}


	for(const auto& obj : o.Objects)
	{
		cout << endl << "***** BEGIN OF OBJECT *****" << endl;
		cout << *obj << endl;
		cout <<  "***** END OF OBJECT *****" << endl;
	}


	cout << "printing active coalitions" << endl;

	for(const auto& coal : o.active_coalitions)
	{
		cout << endl << "***** BEGIN OF COALITION  *****" << endl;
		cout << coal;
		cout << endl << "***** END OF COALITION  *****" << endl;
	}
	cout << endl << "**************** END OF TASK ALLOCATION ****************" << endl;
}

