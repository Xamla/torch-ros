local ros = require 'ros'
require 'ros.actionlib.ActionServer'
local actionlib = ros.actionlib

ros.init('test_action_server')
nh = ros.NodeHandle()
ros.console.setLoggerLevel('actionlib', ros.console.Level.Debug)

local as = actionlib.ActionServer(nh, 'test_action', 'actionlib/Test')

local function ActionServer_Goal(goal_handle)
  ros.INFO("ActionServer_Goal")
end

local function ActionServer_Cancel(goal_handle)
  ros.INFO("ActionServer_Cancel")
  goal_handle:setCanceled(nil, 'blub')
end

as:registerGoalCallback(ActionServer_Goal)
as:registerCancelCallback(ActionServer_Cancel)

print('Starting action server...')
as:start()

while ros.ok() do
  ros.spinOnce()
  sys.sleep(0.01)
end

as:shutdown()
nh:shutdown()
ros.shutdown()
