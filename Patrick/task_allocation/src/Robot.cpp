/*
 * Robot.cpp
 *
 *  Created on: 12 mars 2017
 *      Author: Patrick
 */

#include "Robot.h"

void task_allocation::Get_the_coordination_allocation(int index, double& x)
{
	x=Robots_[index].tau_;
}
void task_allocation::Get_the_coordination_parameter(double& x)
{
	x=Vobject_.gamma_;
}






