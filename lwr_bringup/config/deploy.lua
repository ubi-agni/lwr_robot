require "rttlib"
require "rttros"

tc=rtt.getTC()
d=tc:getPeer("Deployer")

d:import("rtt_rosnode")
-- Start of user code imports
d:import("lwr_fri")

-- End of user code

ros=false
prio=0
schedpol=rtt.globals.ORO_SCHED_OTHER

local opttab=utils.proc_args(arg)
local cp=rtt.Variable("ConnPolicy")

function conn2ros(depl, port, topic)
   depl:stream(port,rtt.provides("ros"):topic(topic))
end


d:loadComponent("FRI", "FRIComponent")
d:setActivity("FRI", 0.0, 5, rtt.globals.ORO_SCHED_RT)
FRI = d:getPeer("FRI")
FRI:getProperty("fri_port"):set(49938)
FRI:configure()





-- ROS integration
d:import("rtt_actionlib")
d:import("rtt_actionlib_msgs")
d:import("rtt_rosnode")
d:import("rtt_roscomm")
d:import("rtt_std_msgs")
d:import("rtt_sensor_msgs")
d:import("rtt_diagnostic_msgs")
d:import("rtt_control_msgs")
d:import("rtt_trajectory_msgs")

--d:import("rtt_dot_service")
--d:loadService("Deployer","dot")

-- Start of user code imports
d:import("internal_space_spline_trajectory_generator")
d:import("internal_space_spline_trajectory_action")


d:loadComponent("JntTrajGen","InternalSpaceSplineTrajectoryGenerator")
d:setActivity("JntTrajGen", 0.001, 5, rtt.globals.ORO_SCHED_RT)
JntTrajGen = d:getPeer("JntTrajGen")
number_of_joints=JntTrajGen:getProperty("number_of_joints")
number_of_joints:set(7)
JntTrajGen:configure()

d:loadComponent("JntTrajAction", "InternalSpaceSplineTrajectoryAction")
d:setActivity("JntTrajAction", 0.1, 2, rtt.globals.ORO_SCHED_RT)
JntTrajAct = d:getPeer("JntTrajAction")
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
  joint_names[i]="lwr_arm_"..i.."_joint"
  lower_limits[i] = lowlims[i+1]
  upper_limits[i] = uplims[i+1]
end 


JntTrajAct:loadService("actionlib")
JntTrajAct:provides("actionlib"):connect("/".."/arm_trajectory_controller/follow_joint_trajectory")


JntTrajAct:configure()

d:addPeer("JntTrajAction","JntTrajGen")

d:connect("JntTrajAction.trajectoryPtr", "JntTrajGen.trajectoryPtr", rtt.Variable("ConnPolicy"))

d:connect("JntTrajGen.JointPositionCommand", "FRI.JointPositionCommand", rtt.Variable("ConnPolicy"))
d:connect("FRI.JointPosition", "JntTrajAction.JointPosition" , rtt.Variable("ConnPolicy"))
d:connect("FRI.JointPosition", "JntTrajGen.JointPosition" , rtt.Variable("ConnPolicy"))

-- ROS in out
ros=rtt.provides("ros")
d:stream("JntTrajAction.command",ros:topic("/".."/arm_trajectory_controller/command"))
d:stream("JntTrajAction.state",ros:topic("/".."/arm_trajectory_controller/state"))


print("finished starting trajectory controller")

-- Start of user code usercode
FRI:start()
JntTrajAct:start()
-- End of user code

