require "rttlib"
require "rttros"

tc=rtt.getTC()
d=tc:getPeer("Deployer")
d:import("rtt_rosnode")

ros=false

-- ROS integration
d:import("rtt_roscomm")
d:import("rtt_rospack")
ros = rtt.provides("ros")

-- node graph
--d:import("rtt_dot_service")
--d:loadService("Deployer","dot")

-- deploy kuka controller
lwr_bringup_path = ros:find("lwr_bringup")
assert(loadfile(lwr_bringup_path.."/scripts/kuka_deploy.lua"))("lwr",49938)

-- deploy trajectory controller
assert(loadfile(lwr_bringup_path.."/scripts/traj_deploy.lua"))("lwr")

-- connect the components together
d:connect("lwrJntTrajAction.trajectoryPtr", "lwrJntTrajGen.trajectoryPtr", rtt.Variable("ConnPolicy"))
d:connect("lwrJntTrajGen.JointPositionCommand", "lwrFRI.JointPositionCommand", rtt.Variable("ConnPolicy"))
d:connect("lwrFRI.JointPosition", "lwrJntTrajGen.JointPosition" , rtt.Variable("ConnPolicy"))
d:connect("lwrFRI.JointPosition", "lwrJntTrajAction.JointPosition" , rtt.Variable("ConnPolicy"))

print("finished starting kuka and trajectory controllers")

-- access previously deployed components
JntTrajAct = d:getPeer("lwrJntTrajAction")

-- Start of user code usercode
JntTrajAct:start()
-- End of user code

