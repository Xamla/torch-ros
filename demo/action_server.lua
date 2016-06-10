local ros = require 'ros'
require 'ros.actionlib.ActionServer'
local actionlib = ros.actionlib


local function ActionServer_Goal(goal_handle)
  ros.INFO("ActionServer_Goal")
  local g = goal_handle:getGoal()
  print(g)
  goal_handle:setAccepted('yip')

  local r = goal_handle:createResult()
  r.result = 123
  print(r)
  --goal_handle:setAborted(r, 'no')
  goal_handle:setSucceeded(r, 'done')
end


local function ActionServer_Cancel(goal_handle)
  ros.INFO("ActionServer_Cancel")
  goal_handle:setCanceled(nil, 'blub')
end


local function testActionServer()
  ros.init('testActionServer')
  nh = ros.NodeHandle()
  ros.console.setLoggerLevel('actionlib', ros.console.Level.Debug)

  local as = actionlib.ActionServer(nh, 'test_action', 'actionlib/Test')

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
end


testActionServer()
