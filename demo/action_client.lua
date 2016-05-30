local ros = require 'ros'
require 'ros.actionlib.SimpleActionClient'
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState

ros.init('test_action_client')
nh = ros.NodeHandle()
ros.console.setLoggerLevel('actionlib', ros.console.Level.Debug)

local ac = actionlib.SimpleActionClient('actionlib/Test', 'test_action', nh)

function test_sync_api()
  local g = ac:createGoal()
  g.goal = 123
  local state = ac:sendGoalAndWait(g, 5, 5)
  ros.INFO('Finished with states: %s (%d)', SimpleClientGoalState[state], state)
  local result = ac:getResult()
  ros.INFO('Result:\n%s', result)
end


function test_async_api()
  -- test async api
  local done = false

  function Action_done(state, result)
    ros.INFO('Action_done')
    ros.INFO('Finished with states: %s (%d)', SimpleClientGoalState[state], state)
    ros.INFO('Result:\n%s', result)
    done = true
  end

  function Action_active()
    ros.INFO('Action_active')
  end

  function Action_feedback(feedback)
    ros.INFO('Action_feedback')
  end

  local g2 = ac:createGoal()
  g2.goal = 456
  ac:sendGoal(g2, Action_done, Action_active, Action_feedback)

  while ros.ok() and not done do
    ros.spinOnce()
  end
end


print('waiting for server connection...')
if ac:waitForServer(ros.Duration(5.0)) then
  print('connected.')

  test_sync_api()
  test_async_api()

else
  print('failed.')
end


ac:shutdown()
nh:shutdown()
ros.shutdown()
