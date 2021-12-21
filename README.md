# zm_local_planner

- zm_local_planner is a simple local planner to follow global planner for a Differential mobile robot.

- Software: Robot Operating System.

- Version: kinetic, melodic, noetic.

- Step1. Download zm_local_planner github link.

``` bash
$ cd <catkin_ws>/src
```

``` bash
$ git clone -b diff https://github.com/qaz9517532846/zm_local_planner.git
```

``` bash
$ cd ..
```

``` bash
$ catkin_make
```

- Step2. zm_local_planner add to move_base.launch file.

``` bash
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <param name="base_local_planner" value="zm_local_planner/ZMLocalPlanner" />
    <rosparam file="$(find zm_robot_navigation)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find zm_robot_navigation)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find zm_robot_navigation)/param/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find zm_robot_navigation)/param/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find zm_robot_navigation)/param/move_base_params.yaml" command="load" />
    <rosparam file="$(find zm_local_planner)/param/zm_local_planner_params.yaml" command="load" />
    <remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>
    <remap from="odom" to="$(arg odom_topic)"/>
  </node>
```

------

## Reference:

[1]. rto_core. https://github.com/dietriro/rto_core

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2021 ZM Robotics Software Laboratory.
