#ifndef COMMON_H
#define COMMON_H

const int N_ROB = 4;
const int N_OBJ = 4;

const double OBJECT_MAX_X = 3.5; // end of conveyor

const int MAX_COALITION_SIZE = 6;
const int MAX_TASKS = 6;

const double OBJ_MAX_PREDICTIONTIME = 100; // effectively useless


// somehow setting SIM_VELOCITY to 30 equates to speed in object.cpp to be between 1 and 2.5...
const double SIM_VELOCITY = 8; // only used by ball.cpp. Makes it easier for us to have it here...

const double X_INIT = -3;
const double Y_INIT = -0.55;
const double Z_INIT = 0.5;



enum class Object_scenarios {ONE, TWO, THREE};
const int N_SCENARIOS = 3;
enum class Object_sizes {SMALL, LARGE};
const Object_scenarios SCENARIO = Object_scenarios::THREE;
const Object_sizes obj_sizes[N_SCENARIOS][N_OBJ] = {{Object_sizes::LARGE, Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL},
                                                   {Object_sizes::LARGE, Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL},
                                                   {Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL}};




#endif // COMMON_H
