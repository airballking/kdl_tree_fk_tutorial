# kdl_tree_fk_tutorial
A small tutorial showing how to calculate forward kinematics for a kinematic tree using KDL.

## Installation
Using ```catkin_tools``` and ```wstool``` in a new workspace for ```ROS Indigo```:
```
source /opt/ros/indigo/setup.bash          # start using ROS Indigo
mkdir -p ~/my_ws/src                       # create directory for workspace
cd ~/my_ws                                 # go to workspace directory
catkin init                                # init workspace
cd src                                     # go to source directory of workspace
wstool init                                # init rosinstall
wstool merge https://raw.githubusercontent.com/airballking/kdl_tree_fk_tutorial/master/rosinstall/catkin.rosinstall
                                           # update rosinstall file
wstool update                              # pull source repositories
rosdep install --ignore-src --from-paths . # install dependencies available through apt
cd ..                                      # go to workspace directory
catkin build                               # build packages
source ~/my_ws/devel/setup.bash            # source new overlay
```

## Running
To bring up the PR2 visualization in ```RVIZ``` simply run:
```shell
$ roslaunch kdl_tree_fk_tutorial pr2_example.launch
```

You should now see the PR2 in RVIZ. Additionally, the should be the gui of the ```joint_state_publisher``` which you can use to move the PR2 in RVIZ.

## Examine the launch-file
Look into to the launch-file with
```shell
$ cat launch/pr2_example.launch

<launch>
  <include file="$(find pr2_description)/robots/upload_pr2.launch"/>

  <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher">
    <param name="use_gui" value="true" />
  </node>

  <node pkg="kdl_tree_fk_tutorial" type="my_robot_state_publisher" name="my_robot_state_publisher"/>

  <node pkg="rviz" type="rviz" name="rviz" required="true"
        args="-d $(find kdl_tree_fk_tutorial)/config/config.rviz" />

</launch>
```

As you can see, everything is pretty standard. Only exception is the node ```my_robot_state_publisher```. That node is implemented in this package, and calculates the FK for the entire kinematic tree of the robot.
