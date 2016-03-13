local ros = require 'ros'

--[[
make sure robitq force torque sensor node is running, e.g. run:

rosrun robotiq_force_torque_sensor rq_sensor
]]

ros.init('read_ft_sensor')

spinner = ros.AsyncSpinner()
spinner:start()

local nodehandle = ros.NodeHandle()

local WrenchStamped_spec = ros.MsgSpec('geometry_msgs/WrenchStamped')

local wrench_input = nodehandle:subscribe("/wrench", WrenchStamped_spec, 100)

while ros.ok() do
  local msg = wrench_input:read(100)
  print(msg)
end

wrench_input:shutdown()
ros.shutdown()
