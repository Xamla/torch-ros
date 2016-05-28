local ros = require 'ros'
require 'ros.actionlib.ActionServer'
local actionlib = ros.actionlib

ros.init('test_action_server')
nh = ros.NodeHandle()

local as = actionlib.ActionServer(nh, 'test_action', 'actionlib/Test')

print('Starting action server...')
as:start()

while ros.ok() do
  ros.spinOnce()
  sys.sleep(0.01)
end

as:shutdown()
nh:shutdown()
ros.shutdown()
