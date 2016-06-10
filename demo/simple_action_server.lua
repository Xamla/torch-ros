local ros = require 'ros'
require 'ros.actionlib.SimpleActionServer'
local actionlib = ros.actionlib


local function SimpleActionServer_onGoal(as)
  ros.INFO("SimpleActionServer_onGoal")

  local g = as:acceptNewGoal()
  print(g)

  local r = as:createResult()
  r.result = 123
  print(r)
  --as:setAborted(r, 'no')
  as:setSucceeded(r, 'done')
end


local function SimpleActionServer_onPreempt(as)
  ros.INFO("SimpleActionServer_onPreempt")
  as:setPreempted(nil, 'blub')
end


local function testSimpleActionServer()
  ros.init('test_action_server')
  ros.console.setLoggerLevel('actionlib', ros.console.Level.Debug)
  nh = ros.NodeHandle()

  local as = actionlib.SimpleActionServer(nh, 'test_action', 'actionlib/Test')

  as:registerGoalCallback(SimpleActionServer_onGoal)
  as:registerPreemptCallback(SimpleActionServer_onPreempt)

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


testSimpleActionServer()
