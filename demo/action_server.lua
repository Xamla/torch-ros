local ros = require 'ros'
require 'ros.actionlib.SimpleActionServer'
local actionlib = ros.actionlib

ros.init('test_action_server')
nh = ros.NodeHandle()
ros.console.setLoggerLevel('actionlib', ros.console.Level.Debug)

local as = actionlib.SimpleActionServer(nh, 'test_action', 'actionlib/Test')

local function ActionServer_Goal()
  ros.INFO("ActionServer_Goal")

  local g = as:acceptNewGoal()
  print(g)

  local r = as:createResult()
  r.result = 123
  print(r)
  as:setAborted(r, 'no')

  --as:setSucceeded(r, 'done')
end

local function ActionServer_Cancel()
  ros.INFO("ActionServer_Cancel")
  as:setPreempted(nil, 'blub')
end

as:registerGoalCallback(ActionServer_Goal)
as:registerPreemptCallback(ActionServer_Cancel)

print('Starting action server...')
as:start()

while ros.ok() do
  ros.spinOnce()
  sys.sleep(0.01)
end

as:shutdown()
nh:shutdown()
ros.shutdown()
