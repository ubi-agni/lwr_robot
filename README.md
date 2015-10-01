Quick start
-----------

* make sure you have a running kuka real or sim, listening on port 127.0.0.1:49938 
   in sim this command will start gazebo, spawn a robot, instantiate the simulated kuka controller plugin
   
```roslaunch lwr_simulation lwr_gazebo.launch```

* deploy the FRIcomponent and a trajectory controller to send position commands to the robot.

```roslaunch lwr_bringup lwr.launch```

* test the movement of the robot by sending a trajectory goal

```
rostopic pub /lwr/arm_trajectory_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal '{
header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: "1"}, 
goal_id: {stamp: {secs: 0, nsecs: 0}, id: }, 
goal: {
trajectory:
{
header:{ 
  seq: 4,
  stamp: {
    secs: 0,
    nsecs: 0},
  frame_id: ''
  },
joint_names: ['lwr_arm_0_joint', 'lwr_arm_1_joint', 'lwr_arm_2_joint', 'lwr_arm_3_joint', 'lwr_arm_4_joint', 'lwr_arm_5_joint', 'lwr_arm_6_joint'],
points: [ 
   {positions: [1.0,1.0,1.0,1.0,1.0,1.0,1.0],
    velocities: [0.0, 0.0,0.0, 0.0,0.0, 0.0,0.0],
    accelerations: [],
    time_from_start: {
      secs: 2,
      nsecs: 0}
}
]
},
path_tolerance: [],
goal_tolerance: []
}
}'
```
