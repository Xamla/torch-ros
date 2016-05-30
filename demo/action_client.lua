local ros = require 'ros'
require 'ros.actionlib.SimpleActionClient'
local actionlib = ros.actionlib

ros.init('test_action_client')
nh = ros.NodeHandle()
ros.console.setLoggerLevel('actionlib', ros.console.Level.Debug)

local ac = actionlib.SimpleActionClient('actionlib/Test', 'test_action', nh)

print('waiting for server connection...')
if ac:waitForServer(ros.Duration(5.0)) then
  print('connected.')

  local g = ac:createGoal()
  g.goal = 123
  local state = ac:sendGoalAndWait(g, 5, 5)
  print(state)

else
  print('failed.')
end

ac:shutdown()
nh:shutdown()
ros.shutdown()
