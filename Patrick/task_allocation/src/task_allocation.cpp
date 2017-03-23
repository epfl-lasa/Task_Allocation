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
	max_n_objects = 5;
	n_objects = 0;
}

Task_allocation::Task_allocation(double dt_, int n_state_, int max_n_bots_, int max_n_tasks_, MatrixXd A_V_, multiarm_ds* DS_, Object_prediction_type Object_motion)
{
	if(max_n_bots_ < 1)
	{
		cout << "less than 1 robot for initialization, n_bots = " << max_n_bots_ <<  endl;
	}
	max_n_robots = max_n_bots_;

	n_robots = 0;
//	Robots = new Robot_agent[max_n_robots];
//	Robots.reserve((unsigned long)(max_n_robots));
	Robots.clear();


	Multi_ds = DS_;

	if(max_n_tasks_ < 1)
	{
		cout << "less than 1 task for initialization, n_tasks = " << max_n_tasks_ << endl;
	}

	max_n_objects = max_n_tasks_;
	n_objects = 0;
	//Objects = new Object[n_objects];

//	Objects.reserve((unsigned long)(max_n_objects)); // this shouldn't be needed
	Objects.clear();
	//init_coalitions();


	dt = dt_;
	n_state = n_state_;
//	A_V = A_V_;

	n_frames = 0;
	Prediction_model = Object_motion;
	catching_pos_is_found = false;
}

VectorXd Task_allocation::get_object_state(int i)
{
	VectorXd empty_vec;
	empty_vec.resize(n_state);
	empty_vec.setConstant(-1);
	if(i >= 0 && i < max_n_objects)
		return Objects[i].get_X_O();
	else
		return empty_vec;
}

void Task_allocation::predict_motion()
{

	for(int i = 0; i < n_objects; i++)
	{
		//cout << "predicting object " << i << endl;
		Objects[i].dumb_predict_motion();
		//cout << "predicted object " << i << endl;
	}
}


void Task_allocation::clear_coalitions()
{
	// we are beginning, there's no active coalition, all robots are unallocated
	active_coalitions.clear();
	unallocated_robots.clear();
	for(auto& rob : Robots)
	{
		unallocated_robots.push_back(&(rob));
	}

	Coalitions.clear();
}


// computes all coalitions for the unallocated robots.
void Task_allocation::build_coalitions()
{

//	Coalitions.reserve(MAX_COALITION_SIZE);
	Coalitions.clear();
	unallocated_robots.clear();

cout << "build_coalitions: building unallocated_robots ...";
// ****** look for unallocated robots
	for(auto& rob : Robots)
	{
		if(rob.get_assignment() ==  -1)
		{
			unallocated_robots.push_back(&rob);
		}
	}
cout << "... done" << endl;

// ****** make all the possible coalitions
cout << "build_coalitions: making the coalitions ..." << endl;
	int n_bots = unallocated_robots.size();
	for(int i = 0; i < min(MAX_COALITION_SIZE, n_bots); i++)
	{

		// ************* make the matrix with all permutations of the available robots
		unsigned long int number_of_coalitions = 10;
		MatrixXd perm = PermGenerator(n_bots,i+1); // i+1 because in array 0 we store the coalitions of size 1(ie singletons)

		int n_rows = perm.rows();
		int n_cols = perm.cols();
		bool dupe = false;
		int to_remove[n_rows];
		int n_dupes = 0;
		for(int j = 0; j < n_rows; j++)
			to_remove[j] = -1;

		cout << "build_coalitions: finding dupes ...";
		// remove the duplicates, as the order does not matter in our case
		for(int u = 0; u < n_rows-1; u++)
		{

			for(int j = u+1; j < n_rows; j++)
			{
				dupe = false;

				check_dupe(perm.row(u), perm.row(j));
				for(int k = 0; k < n_cols-1; k++)
				{
					for(int t = k+1; t < n_cols; t++)
					{
						if(perm(u,k) == perm(j,t))
						{
							cout << "found dupe at u k j t " << u << " " << k << " " << j << " " << t << endl;
							dupe = true;
						}

					}
				}

				if(dupe == true)
				{
					to_remove[n_dupes] = j;
					n_dupes++;
				}
			}
		}
		cout << " done " << endl;
		cout << "perm matrix for i = " << i << " and n_bots = " << n_bots << endl << perm << endl;

		cout << "rows to remove..." << endl;
		for(int j = 0; j < n_dupes; j++)
		{
			cout << j << " ";
		}
		cout << endl;
		cout << "build_coalitions: removing dupes ...";
		for(int j = 0; j < n_dupes; j++)
		{
			removeRow(perm, to_remove[j]);
		}

		cout << "done " << endl;
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
				if(Objects[k].get_assignment() == false)
					test.add_task(&(Objects[k]));
			}
			test.set_id(i*min(MAX_COALITION_SIZE, n_bots)+j);
			Coalitions[i].push_back(test);
//			cout << "Coalition of size " << i+1 << " number " << j << endl << Coalitions[i][j] << endl;
		}
	}


	// some tests on pointers and addresses and stuff in this part. This is just to confirm something*	int lowest_weight = 100000;
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

	// commented out to be preserved.
/*	Coalitions.reserve(MAX_COALITION_SIZE); // coalitions of 0 robots are pointless, so if max = 3, we want 3 arrays.
	Coalitions.clear();
	// the first level of indices is the coalition size
	for(int i = 0; i < min(MAX_COALITION_SIZE, n_robots); i++)
	{
		unsigned long int number_of_coalitions = 10;
		MatrixXd perm = PermGenerator(n_robots,i+1); // i+1 because in array 0 we store the coalitions of size 1(ie singletons)

		int n_rows = perm.rows();
		int n_cols = perm.cols();
		bool dupe = false;
		int to_remove[n_rows];
		int n_dupes = 0;
		for(int j = 0; j < n_rows; j++)
			to_remove[j] = -1;

		// remove the duplicates, as the order does not matter in our case
		for(int u = 0; u < n_rows-1; u++)
		{
			dupe = false;
			for(int j = u+1; j < n_rows; j++)
			{
				for(int k = 0; k < n_cols-1; k++)
				{
					for(int t = k+1; t < n_cols; t++)
					{
						if(perm(u,k) == perm(j,t))
							dupe = true;
					}
				}
				if(dupe == true)
				{
					to_remove[n_dupes] = j;
					n_dupes++;
				}
			}
		}

		for(int j = 0; j < n_dupes; j++)
		{
			removeRow(perm, to_remove[j]);
		}


		number_of_coalitions = perm.rows();

		Coalitions.push_back( std::vector<Coalition>() );
		Coalitions[i].reserve(number_of_coalitions);

		// the 2nd level is the ID of the coalition within that size
		for(int j = 0; j < number_of_coalitions; j++)
		{

			// make the coalition
			Coalition test;
			for(int k = 0; k < i+1; k++) // add all robots that should be in this coalition.
			{
				test.add_robot(&(Robots[perm(j,k)]));
			}
			for(int k = 0; k < n_objects; k++)
			{
				test.add_task(&(Objects[k]));
			}
			Coalitions[i].push_back(test);
//			cout << "Coalition of size " << i+1 << " number " << j << endl << Coalitions[i][j] << endl;
		}
	}
//	cout << "done adding coalitions" << endl;*/
}

void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}

int Task_allocation::add_robot(Robot_agent bot)
{
	if(n_robots < max_n_robots)
	{
		Robots.push_back(bot);
		n_robots++;
	}
	else
	{
		cout << "error, trying to add a robot but max robots is reached. n_robots " << n_robots << " max_n_robots " << max_n_robots << endl;
	}
	return n_robots;
}

int Task_allocation::add_task(Object task)
{

//	cout << "Task allocation, trying to add an object" << endl;
	if(n_objects < max_n_objects)
	{
		Objects.push_back(task);
		n_objects++;
	}
	else
	{
		cout << "error, trying to add an object but max objects is reached. n_objects " << n_objects << " max_n_objects " << max_n_objects << endl;
	}
//	cout << "Task allocation, done adding an object" << endl;
	return n_objects;
}

bool Task_allocation::set_object_state(int i, VectorXd X, VectorXd DX)
{
	if(i < n_objects)
	{
		Objects[i].set_state(X, DX);

		return true;
	}
	return false;
}


void Task_allocation::allocate()
{
	int n_steps = 5;

	// **************** reset everything



	// we are beginning, there's no active coalition, all robots are unallocated

	//reset_coalitions();
	active_coalitions.clear();
	unallocated_robots.clear();

	for(int i = 0; i < n_objects; i++) // the boundary should be something else....
	{
		// *************** build the coalitions
		cout << "building coalitions" << endl;
		build_coalitions();
		cout << "done building coalitions" << endl;

		if(unallocated_robots.size() < 1)
		{
			cout << "all robots have been allocated" << endl;
			break;
		}


		// ************** evaluate the coalitions
		cout << "evaluating coalitions" << endl;
		evaluate_coalitions();

		std::vector<double> coal_values;
		coal_values.reserve(n_robots*n_robots); // this is just to preallocate and hopefully win some time.
		coal_values.clear(); // might not be needed


		// all these are not constant references, we risk changing them here. In some cases we want to change it though.
		// (when computing coalitional value, we want to change the coalitional value in it)
		for(auto& row : Coalitions)
		{
			for(auto& coal : row)
			{
				coal_values.push_back(coal.compute_value()); // not yet implemented, just empty
			}
		}
		cout << "done evaluating coalitions" << endl;
		// now we have all coalitional values.

		// **************** look for smallest coalitional weight
		cout << "looking for best coalition" << endl;
		int lowest_weight = 100000;
		int temp_weight = lowest_weight;
		Coalition* low_coal = NULL;
		for(auto& row : Coalitions)
		{
			for(auto& coal : row)
			{
				temp_weight = coal.get_weight();
				if(0 < temp_weight && temp_weight < lowest_weight)
				{
					lowest_weight = temp_weight;
					low_coal = &(coal);
		//			cout << "comparing the 2 pointers " << low_coal << " " << coal.print_pointer() << endl;
				}
			}
		}



		if(low_coal != NULL)
		{
			cout << "found best coalition" << endl;
	//		cout << *low_coal << endl;

		// *******************
		// add the corresponding coalition to the active coalitions
		// remove the robots from that coalition from the unallocated robots
		// remove the task from the available tasks
		// set the assigned task in this coalition
		// set the assignment (target task) of the robots in this coalition

			active_coalitions.push_back(*low_coal);
			low_coal->assign();
		}
		else
			cout << "didn't find best coalition" << endl;
	}

}

void Task_allocation::evaluate_coalitions()
{

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

void Task_allocation::check_dupe(VectorXd rowA, VectorXd rowB)
{
	int dupe = 0;
	std::vector<bool> found_items;
	for(int i = 0; i < rowA.cols(); i++)
	{
		for(int j = i; j < rowA.cols(); j++)
		{
			if(rowA(i) == rowA(j))
				found_items.push_back(true);
		}
	}

	cout << "checking for dupe " << endl << rowA << endl << rowB << endl;
	cout << "similarity level: " << found_items.size() << endl;

}
/*template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}*/




std::ostream& operator<< (std::ostream& stream, const Task_allocation& o)
{
	cout << endl << "**************** BEGIN OF TASK ALLOCATION ****************" << endl;

	cout << "found catching position " << o.catching_pos_is_found << endl;
	cout << "n state " << o.n_state << endl;
	cout << "max n robots " << o.max_n_robots << endl;
	cout << "max n objects " << o.max_n_objects << endl;

	cout << "n robots " << o.n_robots << endl;

	for(const auto& rob : o.Robots)
	{
		cout << endl << "***** BEGIN OF ROBOT *****" << endl;
		cout << rob;
		cout << "***** END OF ROBOT *****" << endl;
	}

	cout << "n objects " << o.n_objects << endl;

	for(const auto& obj : o.Objects)
	{
		cout << endl << "***** BEGIN OF OBJECT *****" << endl;
		cout << obj << endl;
		cout <<  "***** END OF OBJECT *****" << endl;
	}



	// print the coalitions. Isn't C++11 amazing?
	cout << "printing all coalitions" << endl;
	for (const auto& inner : o.Coalitions)
	{
	  for (const auto& item : inner)
	  {
		  cout << endl << "***** BEGIN OF COALITION  *****" << endl;
		  cout << item << endl;
		  cout << endl << "***** END OF COALITION  *****" << endl;
	  }
	}


	cout << "printing active coalitions" << endl;

	for(const auto& coal : o.active_coalitions)
	{
		cout << endl << "***** BEGIN OF COALITION  *****" << endl;
		cout << coal << endl;
		cout << endl << "***** END OF COALITION  *****" << endl;
	}
	cout << endl << "**************** END OF TASK ALLOCATION ****************" << endl;
}

