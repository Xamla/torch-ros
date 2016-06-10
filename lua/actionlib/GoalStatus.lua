local ros = require 'ros.env'


-- http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
local GoalStatus = {
  PENDING         = 0,  -- The goal has yet to be processed by the action server
  ACTIVE          = 1,  -- The goal is currently being processed by the action server
  PREEMPTED       = 2,  -- The goal received a cancel request after it started executing
                        --   and has since completed its execution (Terminal State)
  SUCCEEDED       = 3,  -- The goal was achieved successfully by the action server (Terminal State)
  ABORTED         = 4,  -- The goal was aborted during execution by the action server due
                        --    to some failure (Terminal State)
  REJECTED        = 5,  -- The goal was rejected by the action server without being processed,
                        --    because the goal was unattainable or invalid (Terminal State)
  PREEMPTING      = 6,  -- The goal received a cancel request after it started executing
                        --    and has not yet completed execution
  RECALLING       = 7,  -- The goal received a cancel request before it started executing,
                        --    but the action server has not yet confirmed that the goal is canceled
  RECALLED        = 8,  -- The goal received a cancel request before it started executing
                        --    and was successfully cancelled (Terminal State)
  LOST            = 9   -- An action client can determine that a goal is LOST. This should not be
                        --    sent over the wire by an action server
}
ros.actionlib.GoalStatus = GoalStatus


return GoalStatus
