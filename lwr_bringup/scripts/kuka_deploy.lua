require "rttlib"
require "rttros"

local pathOfThisFile =debug.getinfo(1,"S").source:match[[^@?(.*[\/])[^\/]-$]]

if pathOfThisFile then
  print("path of this file :"..pathOfThisFile)
  package.path = pathOfThisFile..'?.lua;' .. package.path 
end

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

-- get the prefix and port from args
local prefix=""
local prefix_arg,port_arg=...
if prefix_arg then
  print("using prefix "..prefix_arg)
  prefix = prefix_arg
end

if port_arg then
  print("using port "..port_arg)
  port = port_arg
else
  port = 49938
end

-- create LuaComponent
name = prefix.."kuka_controller"

d:import("lwr_fri")
d:import("oro_joint_state_publisher")
d:import("rtt_sensor_msgs")
d:import("rtt_diagnostic_msgs")

namespace = prefix
  
diagname = namespace.."LWRDiag"
d:loadComponent(diagname, "FRIDiagnostics")
d:setActivity(diagname, 0.01, 2, rtt.globals.ORO_SCHED_RT)
diag = d:getPeer(diagname)
diag:configure()
-- add dia to the parent component peers
d:addPeer(tcName, diagname)        

-- deploy FRI and advertize its input/output
friname = namespace.."FRI"
d:loadComponent(friname, "FRIComponent")
d:setActivity(friname, 0, 80, rtt.globals.ORO_SCHED_RT)
fri = d:getPeer(friname)
fri:getProperty("fri_port"):set(port)
fri:configure()
  
-- add fri to the parent component peers
d:addPeer(tcName, friname)  

-- deploy joint state publisher
jspname = namespace.."JntPub"
d:loadComponent(jspname, "JointStatePublisher")
d:setActivity(jspname, 0.01, 10, rtt.globals.ORO_SCHED_RT)
jsp = d:getPeer(jspname)
joint_names = jsp:getProperty("joint_names")
joint_names:get():resize(7)
for i=0,6,1 do
  joint_names[i] = namespace.."_arm_"..i.."_joint"
end
jsp:configure()
-- add jsp to the parent component peers
d:addPeer(tcName, jspname)

-- internal connection
d:connect(friname..".RobotState", diagname..".RobotState", rtt.Variable("ConnPolicy"))
d:connect(friname..".FRIState", diagname..".FRIState", rtt.Variable("ConnPolicy"))
d:connect(friname..".JointPosition", jspname..".JointPosition", rtt.Variable("ConnPolicy"))
d:connect(friname..".JointVelocity", jspname..".JointVelocity", rtt.Variable("ConnPolicy"))
d:connect(friname..".JointTorque", jspname..".JointEffort", rtt.Variable("ConnPolicy"))

-- ROS in out
local ros=rtt.provides("ros")
d:stream(diagname..".Diagnostics",ros:topic(namespace.."/diagnostics"))
d:stream(jspname..".joint_state",ros:topic(namespace.."/joint_states"))

print(namespace.."kuka_controller configured")

-- stat the components
fri:start()
diag:start()
jsp:start()

print("finished loading "..prefix.."kuka")
