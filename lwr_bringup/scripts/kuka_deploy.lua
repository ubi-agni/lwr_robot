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

-- get the prefix
local prefix=""
local prefix_arg=...
if prefix_arg then
  print("using prefix "..prefix_arg)
  prefix = prefix_arg
end
d:import("agni_rtt_services")

-- create LuaComponent
name = prefix.."kuka_controller"
d:loadComponent(name, "OCL::LuaComponent")
d:addPeer(name, "Deployer")
-- ... and get a handle to it
local kuka_controller = d:getPeer(name)
-- add service lua to new component named name
d:loadService(name,"Lua")
 
-- load the Lua hooks
kuka_controller:exec_file(pathOfThisFile.."kuka_controller.lua")

-- configure the component
kuka_controller:getProperty("namespace"):set(prefix)
kuka_controller:getProperty("port"):set(49938)
kuka_controller:getProperty("controller_name"):set(prefix.."/kuka_controller")

kuka_controller:configure()

-- stat the component
kuka_controller:start()

print("finished loading "..prefix.."kuka")
