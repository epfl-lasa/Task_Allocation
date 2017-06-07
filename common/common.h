#ifndef COMMON_H
#define COMMON_H

const int N_ROB = 4;
const int N_OBJ = 4;

const double OBJECT_MAX_X = 3.5; // end of conveyor
const double OBJECT_MIN_X = -1; // begin of robot workspace

const int MAX_COALITION_SIZE = 2;


const int N_CHECKPOINTS = 2;
const double X_checkpoint[N_CHECKPOINTS] = {0, 2};

const double ROBOT_MAX_COST = 1000000; // maximum costs for the robots, this guarantees they wont allocate if the value of the objects doesn't reach higher than this.


const double SIM_VELOCITY[][N_OBJ] = {{0.6,0.6,0.6,0.6},{0.0,0.0,0.0,0.0}}; // units of grid/s. only used by ball.cpp. Makes it easier for us to have it here...

const double LIFT_VELOCITY = 10;
const double X_INIT = -3;
const double Y_INIT = -0.625;
const double Z_INIT = 0.5;



enum class Object_scenarios {ONE, TWO, THREE};
const int N_SCENARIOS = 3;
enum class Object_sizes {SMALL, LARGE};
const Object_scenarios SCENARIO = Object_scenarios::THREE;

// this array should be of size  [N_SCENARIOS][N_OBJ] at least
const Object_sizes obj_sizes[][N_OBJ] = {{Object_sizes::LARGE, Object_sizes::SMALL, Object_sizes::SMALL,  Object_sizes::SMALL},
                                                   {Object_sizes::LARGE, Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL},
                                                   {Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL}};


const double MID_BELT = -0.625;
const double Y_BELT = 0.3;

/*const Object_sizes scenario1[] = {Object_sizes::LARGE, Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL};
const Object_sizes scenario2[] = {Object_sizes::LARGE, Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL};
const Object_sizes scenario3[] = {Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL};

const Object_sizes* obj_sizes[] = {scenario1, scenario2, scenario3};
*/


const double SIGMOID_SLOPE_FACTOR = 100;
const double OBJ_MIN_SPEED = 0.05;
const double OBJ_MAX_SPEED = 3;



inline double sigmoid(double x)
{
    return 1.0f/(1+exp(-SIGMOID_SLOPE_FACTOR*x));
}

#endif // COMMON_H
