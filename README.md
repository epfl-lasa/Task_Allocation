# IJRR
 To fix the positions of the robots with respect to each other:
 
 1- Run and make sure that the transformation matrix is correct!
 ```
rosrun position_of_the_robots position_of_the_robots_node
```
2- Change X[0]=;Y[0]=;Z[0]=; accordingly. X[0] is for KUKA 14 and X[1] is for KUKA 7. KUKA 14 is on the left side of KUKA 7 hence, the left kuka is KUKA 14 and right kuka is kuka 7. The coordination frame is located at the base of KUKA 7 and everything is calculated with respect to this frame.

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
or on the beast
```
rosrun rqt_gui rqt_gui --perspective-file  /home/sina/catkin_workspace/src/IJRR/kuka-rviz-simulation/kuka_lwr_bringup/rqt_config/rqt_gui_bimanual.perspective 
```

####To run the Gripper
```
rosrun robotiq_s_model_control SModelTcpNode.py 192.168.1.11
```
####To run Allegro hand
```
rosrun allegrolib allegrolib
```


####Instructions same as always:
```
>> job
>> init
>> catch
```

## You need to copy these files:
```
scp -r /home/sina/catkin_workspace/src/IJRR/simple_example_two_kuka/data  sina@192.168.100.2:/home/sina/catkin_workspace/src/IJRR/simple_example_two_kuka/data
scp -r /home/sina/catkin_workspace/src/IJRR/intercept_finder/  sina@192.168.100.2:/home/sina/catkin_workspace/src/IJRR
scp -r /home/sina/catkin_workspace/src/IJRR/object_recognition/  sina@192.168.100.2:/home/sina/catkin_workspace/src/IJRR
```
## IP settings:

The IP of the beast:

- export ROS_IP=192.168.100.1
-  export ROS_MASTER_URI=http://192.168.100.1:11311
- KUKA 14: 192.170.10.3 (Connected to the motherboard)
- Hub : 192.168.100.1 (Connected to the PCI ethernet card)
- Internet : 128.178.145.215 (Connected to the ethernet/usb)



The IPs of the kinetect pc!

- export ROS_IP=192.168.100.2
- export ROS_MASTER_URI=http://192.168.100.1:11311
- KUKA 7: 192.170.10.1 (Connected to the motherboard)
- Hub : 192.168.100.2 (Connected to the PCI ethernet card)


