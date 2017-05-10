#ifndef COMMON_H
#define COMMON_H

const int N_ROB = 4;
const int N_OBJ = 4;

const double OBJECT_MAX_X = 3.5; // end of workspace

const int MAX_COALITION_SIZE = 6;
const int MAX_TASKS = 6;

const double OBJ_MAX_PREDICTIONTIME = 100; // effectively useless



enum class Object_scenarios {ONE, TWO, THREE};
const int N_SCENARIOS = 3;
enum class Object_sizes {SMALL, LARGE};
const Object_scenarios SCENARIO = Object_scenarios::ONE;
const Object_sizes obj_sizes[N_SCENARIOS][N_OBJ] = {{Object_sizes::LARGE, Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL},
                                                   {Object_sizes::LARGE, Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL},
                                                   {Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL, Object_sizes::SMALL}};




#endif // COMMON_H
