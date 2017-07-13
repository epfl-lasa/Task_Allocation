# Task allocation
This code goes with the Multi-robot arm system task allocation project.
It simulates multiple objects on a conveyor belt and robots placed along it should take those objects off the conveyor.


# Running the simulation

Launch ros
```
roscore
```

Run task allocation
```
rosrun task_allocation_node task_allocation_node
```

Multi-arm
```
roslaunch kuka_lwr_bringup multiarm_simulation.launch
```

Run robot_simulator module from ~/catkin_ws/src/robot-toolkit
```
./bin/robot_simulator --config packages/simple_example_two_kuka/Scenario
```

Bringup RQT_gui simulation
```
rosrun rqt_gui rqt_gui --perspective-file  /home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/IJRR/kuka-rviz-simulation/kuka_lwr_bringup/rqt_config/rqt_gui_bimanual.perspective
```


Instructions same as always:
```
>> job
>> init
>> catch
```

Start/stop command added which toggles the run mode (starts/stops the task allocation and object motion).

```
>> stop 
```


# Code info

Constants from the report are kind of magic numbers. Should be moved to common.h ideally.

Task allocation node has while loop so that it "starts" at "init" where it receives info from robot_simulator.


## Scenarios
Object color is defined in ball.cpp

### common.h
To run a different scenario: modify SCENARIO to Object_scenarios::ONE, TWO or THREE.
Objects velocities are defined in sim_velocities[2][N_obj]. First one is X, second is Y. For each object.
Objects can be small or large, which defines the size of the cube and a few other things (number of grasping positions etc)
Object scenarios are placed with respect to object 0 currently...



## Objects
Evaluates own trajectory and value. Uses "dumb_predict_motion()". Position gets assigned by callback.

## Robots
Evaluates cost of objects. Position gets assigned by callbacks (!)

## Coalition
nothing particular.

## Task allocation
Contains the allocation algorithm and other high level stuff

## Task allocation node
This runs basic stuff:
1) Object trajectory prediction
2) Allocation
3) Allocation publication
