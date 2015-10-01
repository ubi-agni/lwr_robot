require "rttlib"
require "rttros"

tc=rtt.getTC()
tcName=tc:getName()
-- find the deployer
-- script might be started from deployer directly or from any other component, hopefully having a deployer as peer
if tcName=="Deployer" then
  d=tc
else
  d=tc:getPeer("Deployer")
    -- TODO complain and exit if deployer not found
end

-- get the prefix from args
local prefix=""
local prefix_arg=...
if prefix_arg then
  print("using prefix "..prefix_arg)
  prefix = prefix_arg
end

-- Dependencies
d:import("rtt_actionlib")
d:import("rtt_actionlib_msgs")
d:import("rtt_trajectory_msgs")
d:import("rtt_control_msgs")

-- Start of user code imports
d:import("internal_space_spline_trajectory_generator")
d:import("internal_space_spline_trajectory_action")

-- JointTrajectory Generator
jtg_name = prefix.."JntTrajGen"
d:loadComponent(jtg_name,"InternalSpaceSplineTrajectoryGenerator")
d:setActivity(jtg_name, 0.001, 5, rtt.globals.ORO_SCHED_RT)
JntTrajGen = d:getPeer(jtg_name)
number_of_joints=JntTrajGen:getProperty("number_of_joints")
number_of_joints:set(7)
JntTrajGen:configure()

-- JointTrajectory Actionlib
jta_name = prefix.."JntTrajAction"
d:loadComponent(jta_name, "InternalSpaceSplineTrajectoryAction")
d:setActivity(jta_name, 0.1, 2, rtt.globals.ORO_SCHED_RT)
JntTrajAct = d:getPeer(jta_name)
nbr_of_joints=JntTrajAct:getProperty("number_of_joints")
nbr_of_joints:set(7)
joint_names=JntTrajAct:getProperty("joint_names")
lower_limits=JntTrajAct:getProperty("lower_limits")
upper_limits=JntTrajAct:getProperty("upper_limits")
joint_names:get():resize(7)
lower_limits:get():resize(7)
upper_limits:get():resize(7)
-- TODO move that into a yaml and load it via the launch files that also spawns the lua
lowlims = {-2.96 , -2.09, -2.96, -2.09, -2.96, -2.09, -2.96}
uplims = {2.96 , 2.09, 2.96, 2.09, 2.96, 2.09, 2.96}
for i=0,6,1 do 
  joint_names[i]=prefix.."_arm_"..i.."_joint"
  lower_limits[i] = lowlims[i+1]
  upper_limits[i] = uplims[i+1]
end 
JntTrajAct:loadService("actionlib")
JntTrajAct:provides("actionlib"):connect("/"..prefix.."/arm_trajectory_controller/follow_joint_trajectory")
-- ROS in out
local ros=rtt.provides("ros")
d:stream(jta_name..".command",ros:topic("/"..prefix.."/arm_trajectory_controller/command"))
d:stream(jta_name..".state",ros:topic("/"..prefix.."/arm_trajectory_controller/state"))
JntTrajAct:configure()

d:addPeer(jta_name,jtg_name)
