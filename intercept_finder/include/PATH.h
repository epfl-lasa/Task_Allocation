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

#include <iomanip>
#include <locale>
#include <sstream>
#include <string> // this should be already included in <sstream>




using namespace std;
//std::string Commom_path  = "/home/sina/catkin_workspace/src/IJRR/simple_example_two_kuka/data";
std::string Common_path  = "/home/patrick/lasa-project/catkin_ws/src/IJRR/simple_example_two_kuka/data";

// Path to SVM Collision Model
std::string svm_filename = "/home/patrick/lasa-project/catkin_ws/src/IJRR/SVMGrad/SVMGrad/matlab/models/FenderwHands/36D-29k-Optimal-Model-FenderwHands.txt";

const int N_robots=4;	//You need to do some changes if you want to put it 1!
const int N_grabbing=2; //You need to do some changes if you want to put it 1!

enum ENUM_COMMAND{COMMAND_INITIAL=0,COMMAND_JOB,COMMAND_Grab,COMMAND_NONE};
double dt=0.002;


string addTwochar(string folder_path, string b, int integer1=-1,int integer2=-1)
{
	std::string str;
	str.append(folder_path);
	str.append(b);
	if (integer1!=-1)
	{
		ostringstream convert;
		convert << integer1;
		str.append(convert.str());
	}
	if (integer2!=-1)
	{
		ostringstream convert;
		convert << integer2;
		str.append(convert.str());
	}
	str.append(".txt");
	return str;
}
string addTwostring(string string1, string string2, int integer1)
{
	std::string str;
	str.append(string1);
	str.append(string2);
	ostringstream convert;
	convert << integer1;
	str.append(convert.str());
	return str;
}
