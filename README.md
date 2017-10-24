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
