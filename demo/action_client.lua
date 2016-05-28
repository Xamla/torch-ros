local ros = require 'ros'
require 'ros.actionlib.SimpleActionClient'
local actionlib = ros.actionlib

ros.init('test_action_client')
nh = ros.NodeHandle()

local ac = actionlib.SimpleActionClient('actionlib/Test', 'test_action', nh)

print('waiting for server connection...')
if ac:waitForServer(ros.Duration(5.0)) then
  print('connected.')
else
  print('failed.')
end

ac:shutdown()
nh:shutdown()
ros.shutdown()
