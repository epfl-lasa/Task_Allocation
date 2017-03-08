/*
  Copyright (c) 2013 Sina Mirrazavi,
  LASA Lab, EPFL, CH-1015 Lausanne, Switzerland,
  http://lasa.epfl.ch

  The program is free for non-commercial academic use.
  Please acknowledge the authors in any academic publications that have
  made use of this code or part of it.
  }
*/

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <time.h>
#include <string>
#include <fstream>
#include "ros/ros.h"
#include "MathLib.h"
#include "geometry_msgs/Pose.h"
#include  "std_msgs/Int64.h"
#include "std_msgs/Float32MultiArray.h"
#include "visualization_msgs/InteractiveMarkerFeedback.h"
#include <sstream>
#include "sg_filter.h"



using namespace std;
using namespace MathLib;



enum Command{Com_Ball_INIT=0, Com_Ball_Move};
enum ENUM_State{Com_Stop,Com_Break, Com_Safe};
enum ENUM_COMMAND{COMMAND_INITIAL=0,COMMAND_JOB,COMMAND_Grab,COMMAND_NONE};


Vector P_O;
Vector Shift_left_P_O;
Vector Shift_right_P_O;
Vector DP_O;
Vector DDP_O;
double dt;


int order = 5;
int winlen = 50;
SGF::real sample_time = 0.005;
int dim=3;
SGF::Vec inp(3);
SGF::Vec outp(3);
int ret_code;
SGF::SavitzkyGolayFilter *filter;
