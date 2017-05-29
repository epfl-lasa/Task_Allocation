#ifndef COMMON_H
#define COMMON_H

const int N_ROB = 4;
const int N_OBJ = 4;

const double OBJECT_MAX_X = 3.5; // end of conveyor

const int MAX_COALITION_SIZE = 2;
//const int MAX_TASKS = 20;

const double OBJ_MAX_PREDICTIONTIME = 100; // effectively useless

const int N_CHECKPOINTS = 2;
const double X_checkpoint[N_CHECKPOINTS] = {0, 2};

const double ROBOT_MAX_COST = 1000000; // maximum costs for the robots, this guarantees they wont allocate if the value of the objects doesn't reach higher than this.

// somehow setting SIM_VELOCITY to 30 equates to speed in object.cpp to be between 1 and 2.5...
const double SIM_VELOCITY[] = {0.4,0.4,0.4,0.4}; // only used by ball.cpp. Makes it easier for us to have it here...

const double X_INIT = -3;
const double Y_INIT = -0.55;
const double Z_INIT = 0.5;



enum class Object_scenarios {ONE, TWO, THREE};
const int N_SCENARIOS = 3;
enum class Object_sizes {SMALL, LARGE};
const Object_scenarios SCENARIO = Object_scenarios::ONE;
const Object_sizes obj_sizes[N_SCENARIOS][N_OBJ] = {{Object_sizes::LARGE, Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL},
                                                   {Object_sizes::LARGE, Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL},
                                                   {Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL}};

const double SIGMOID_SLOPE_FACTOR = 100;
const double OBJ_MIN_SPEED = 0.05;
const double OBJ_MAX_SPEED = 30;



inline double sigmoid(double x)
{
    return 1.0f/(1+exp(-SIGMOID_SLOPE_FACTOR*x));
}

#endif // COMMON_H
