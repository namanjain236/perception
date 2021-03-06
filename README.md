# Object Localization through Heuristic Search

Overview
--------

This stack provides two packages: kinect_sim and sbpl_perception.
The former is the depth-image renderer/simulator. Given a camera pose, set of 3D models/meshes and their locations,
it can generate a depth-image for that scene. 

The latter is the object localization package and contains the search environment that communicates with the planner.
It internally uses kinect_sim to generate 'states' for the planner to search through.

Setup
-----

1. Get ROS Hydro from http://wiki.ros.org/hydro/Installation/Ubuntu
2. Create a rosbuild workspace ~/hydro_workspace as described in http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment and make sure ~/hydro_workspace/sandbox is in your ROS package path.

```bash
cd ~/hydro_workspace/sandbox && git clone https://github.com/venkatrn/perception.git
cd ~/hydro_workspace/sandbox && git clone https://github.com/venkatrn/improved-mha-planner.git 
rosmake sbpl_perception
#If above fails, try the following to diagnose error:
#rosmake sbpl
#rosmake kinect_sim
```
 If you have any missing ROS package dependencies, do:
 ```bash
 sudo apt-get install ros-hydro-<package> #or replace 'hydro' by appropriate version name
 ```

Test
----

Please download required data files from https://sbpl.pc.cs.cmu.edu/shared/Venkat/sbpl_perception/ and place the data directory under ~/hydro_workspace/sandbox/perception/sbpl_perception/

To test the system, run 
```bash
roslaunch sbpl_perception sim_test.launch 
```

This should dump some output images into your /tmp folder. Upon successful completion of the search, you should see an image named goal_state.png.
