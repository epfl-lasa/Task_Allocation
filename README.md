# Task allocation
This code goes with the Multi-robot arm system task allocation project.
It simulates multiple objects on a conveyor belt and robots placed along it should take those objects off the conveyor.


# Running the simulation:

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
Task allocation node has while loop so that it "starts" at "init" where it receives info from robot_simulator.


## Scenarios

To run a different scenario: modify "Scenario" to ONE, TWO or THREE in common.h.
Object scenarios are placed with respect to object 0 currently...

## Objects
Evaluates own trajectory and value. Uses "dumb_predict_motion()". Position gets assigned by callback.

## Robots
Evaluates cost of objects. Position gets assigned by callbacks (!)

