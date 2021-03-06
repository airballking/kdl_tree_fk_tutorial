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
```
```xml
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

## Examine ```my_robot_state_publisher```
In the shell, run
```shell
$ cat src/my_robot_state_publisher.cpp
```
```cpp
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <robot_state_publisher/treefksolverposfull_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_kdl/tf2_kdl.h>

class MyRobotStatePublisher
{
  public:
    MyRobotStatePublisher(const ros::NodeHandle& nh, const KDL::Tree& tree) : 
      nh_(nh),
      joint_state_sub_( nh_.subscribe("/joint_states", 1, &MyRobotStatePublisher::js_callback, this) ),
      fk_solver_( tree )
    {
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    tf2_ros::TransformBroadcaster tf2_pub_;
    KDL::TreeFkSolverPosFull_recursive fk_solver_;

    void js_callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      // translate joint state message into map
      std::map<std::string, double> joint_state_map;
      if (msg->name.size() != msg->position.size())
        throw std::runtime_error("Received joint state message with fields name and position of unequal lengths.");
      for (size_t i=0; i<msg->name.size(); ++i)
        joint_state_map.insert(std::make_pair(msg->name[i], msg->position[i]));

      // calculate FK results for entire tree
      std::map<std::string, tf2::Stamped<KDL::Frame> > fk_results;
      if (fk_solver_.JntToCart(joint_state_map, fk_results, false) != 0)
        throw std::runtime_error("FK solver returned with an error.");

      // convert FK results to TF2 messages
      std::vector<geometry_msgs::TransformStamped> tf2_msgs;
      for (std::map<std::string, tf2::Stamped<KDL::Frame> >::const_iterator it=fk_results.begin(); it!=fk_results.end(); ++it)
      {
        geometry_msgs::TransformStamped tf2_msg = tf2::kdlToTransform(it->second);
        tf2_msg.header.stamp = it->second.stamp_;
        tf2_msg.header.frame_id = it->second.frame_id_;
        tf2_msg.child_frame_id = it->first;
        tf2_msgs.push_back(tf2_msg);
      }

      // publish data to TF2
      tf2_pub_.sendTransform(tf2_msgs);
    }
};

int main(int argc, char** argv)
{
   ros::init(argc, argv, "my_robot_state_publisher");
   ros::NodeHandle nh("~");

   try
   {
     // read URDF from parameter server and parse it into a KDL::Tree
     KDL::Tree tree;
     if (!kdl_parser::treeFromParam("/robot_description", tree))
       throw std::runtime_error("Could not find robot URDF in parameter '/robot_description'.");

     // start my robot state publisher
     MyRobotStatePublisher state_pub(nh, tree);
     ros::spin();
   }
   catch (const std::exception& e)
   {
     ROS_ERROR("%s", e.what());
   }

   return 0;
}
```

As you can see, the node is actually not very big. Most of the computation is done by the class ```KDL::TreeFkSolverPosFull_recursive```. 

Interestingly, this class is NOT part of ```KDL```, but it is hidden in the ROS package ```robot_state_publisher```where it is no longer used. I found this interesting discussion that might point out why this useful class never made it into ```KDL```: http://www.orocos.org/forum/rtt/rtt-dev/tree-solver-api

