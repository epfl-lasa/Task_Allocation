Running the simulation:

Launch ros
```
roscore
```

#Run task allocation
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



Task allocation "starts" at "init" where it receives info from robot_simulator.



#Run a different scenario: modify "Scenario" to ONE, TWO or THREE in common.h

#Object scenarios are placed with respect to object 0 currently...