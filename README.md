

To fix the illustration:

You need to change the position of the robots and the object with respect to eachother and the world-frame.
Open basic_shapes.cpp and change double B_t_U=; then open "kuka_bimanual_lwr_lasa.urdf.xacro" and change the position of the robots!

Running the simulation:

####Bringup simulation
```
roslaunch kuka_lwr_bringup bimanual_simulation.launch --screen
```

##To move the robots! Run the fri_iiwa_ros interfece on both pcs 
```
rosrun fri_iiwa_ros fri_iiwa_example_ros
```
####Run robot_simulator module from ~/catkin_ws/src/robot-toolkit
```
./bin/robot_simulator --config packages/simple_example_two_kuka/Scenario
```

####Bringup RQT_gui simulation
```
rosrun rqt_gui rqt_gui --perspective-file  /home/sina/Dropbox/Sinas_stuff/catkin_ws/underlay/src/IJRR/kuka-rviz-simulation/kuka_lwr_bringup/rqt_config/rqt_gui_bimanual.perspective
```


####Instructions same as always:
```
>> job
>> init
>> catch
```

