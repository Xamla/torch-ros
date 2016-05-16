local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local actionlib = ros.actionlib

local CommState = {
  WAITING_FOR_GOAL_ACK    = 1,
  PENDING                 = 2,
  ACTIVE                  = 3,
  WAITING_FOR_RESULT      = 4,
  WAITING_FOR_CANCEL_ACK  = 5,
  RECALLING               = 6,
  PREEMPTING              = 7,
  DONE                    = 8,
  'WAITING_FOR_GOAL_ACK', 'PENDING', 'ACTIVE', 'WAITING_FOR_RESULT', 'WAITING_FOR_CANCEL_ACK', 'RECALLING', 'PREEMPTING', 'DONE'
}

-- http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
local ActionLibGoalStatus = {
  PENDING         = 0   -- The goal has yet to be processed by the action server
  ACTIVE          = 1   -- The goal is currently being processed by the action server
  PREEMPTED       = 2   -- The goal received a cancel request after it started executing
                        --   and has since completed its execution (Terminal State)
  SUCCEEDED       = 3   -- The goal was achieved successfully by the action server (Terminal State)
  ABORTED         = 4   -- The goal was aborted during execution by the action server due
                        --    to some failure (Terminal State)
  REJECTED        = 5   -- The goal was rejected by the action server without being processed,
                        --    because the goal was unattainable or invalid (Terminal State)
  PREEMPTING      = 6   -- The goal received a cancel request after it started executing
                        --    and has not yet completed execution
  RECALLING       = 7   -- The goal received a cancel request before it started executing,
                        --    but the action server has not yet confirmed that the goal is canceled
  RECALLED        = 8   -- The goal received a cancel request before it started executing
                        --    and was successfully cancelled (Terminal State)
  LOST            = 9   -- An action client can determine that a goal is LOST. This should not be
                        --    sent over the wire by an action server
}

local SimpleGoalState = {
  PENDING                 = 1,
  ACTIVE                  = 2,
  DONE                    = 3,
  'PENDING', 'ACTIVE', 'DONE'
}

local SimpleClientGoalState = {
  PENDING                 = 1,
  ACTIVE                  = 2,
  RECALLED                = 3,
  REJECTED                = 4,
  PREEMPTED               = 5,
  ABORTED                 = 6,
  SUCCEEDED               = 7,
  LOST                    = 8,
  'PENDING', 'ACTIVE', 'RECALLED', 'REJECTED', 'PREEMPTED', 'ABORTED', 'SUCCEEDED', 'LOST'
}

local TerminalState = {
  RECALLED                = 1,
  REJECTED                = 2,
  PREEMPTED               = 3,
  ABORTED                 = 4,
  SUCCEEDED               = 5,
  LOST                    = 6,
  'RECALLED', 'REJECTED', 'PREEMPTED', 'ABORTED', 'SUCCEEDED', 'LOST'
}

local next_goal_id = 1    -- shared among all action clients
local ActionClient = torch.class('ros.actionlib.ActionServer', actionlib)

local function goalConnectCallback(self, name, topic)
  self.goalSubscribers[name] = (self.goalSubscribers[name] or 0) + 1
  ros.DEBUG_NAMED("ActionClient", "goalConnectCallback: Adding [%s] to goalSubscribers", name)
end

local function goalDisconnectCallback(self, name, topic)
  local count = self.goalSubscribers[name]
  if count == nil then
    -- should never happen, copied warning from official actionlib impl
    ros.WARN_NAMED("ActionClient", "goalDisconnectCallback: Trying to remove [%s] from goalSubscribers, but it is not in the goalSubscribers list.", name)
  else
    ros.DEBUG_NAMED("ActionClient", "goalDisconnectCallback: Removing [%s] from goalSubscribers, (remaining with same name: %d)", name, count - 1)
    if count <= 1 then
      self.goalSubscribers[name] = nil
    else
      self.goalSubscribers[name] = count - 1
    end
  end
end

local function cancelConnectCallback(self, name, topic)
  self.cancelSubscribers[name] = (self.cancelSubscribers[name] or 0) + 1
  ros.DEBUG_NAMED("ActionClient", "cancelConnectCallback: Adding [%s] to cancelSubscribers", name)
end

local function cancelDisconnectCallback(self, name, topic)
  local count = self.cancelSubscribers[name]
  if count == nil then
    -- should never happen, copied warning from official actionlib impl
    ros.WARN_NAMED("ActionClient", "cancelDisconnectCallback: Trying to remove [%s] from cancelSubscribers, but it is not in the cancelSubscribers list.", name)
  else
    ros.DEBUG_NAMED("ActionClient", "cancelDisconnectCallback: Removing [%s] from cancelSubscribers (remaining with same name: %d)", name, count - 1)
    if count <= 1 then
      self.cancelSubscribers[name] = nil
    else
      self.cancelSubscribers[name] = count - 1
    end
  end
end

local function transitionToState(self, goal, next_state)
  ros.DEBUG_NAMED("ActionClient", "Transitioning CommState from %s to %s", CommState[goal.state], CommState[next_state])
  goal.state = next_state
  if goal.transition_cb ~= nil then
    goal.transition_cb(goal)
  end
end

local function processLost(self, goal)
  ros.WARN_NAMED("ActionClient", "Transitioning goal to LOST")
  if goal.latest_goal_status ~= nil then
    goal.latest_goal_status.status = ActionLibGoalStatus.LOST -- LOST
    goal.latest_goal_status.text = "LOST"
  end
  transitionToState(self, goal, CommState.DONE)
end

local function updateStatus(self, goal, goal_status)
  -- check if pending action that it is correctly reflected by the status message
  if goal_status ~= nil then
    goal.latest_goal_status = goal_status
  elseif goal.state ~= CommState.WAITING_FOR_GOAL_ACK and goal.state ~= CommState.WAITING_FOR_RESULT and goal.state != CommState.DONE then
    processLost(self, goal)
    return
  end

  if goal.state == CommState.WAITING_FOR_GOAL_ACK then

    if goal_status.status == ActionLibGoalStatus.PENDING then
      transitionToState(self, goal, CommState.PENDING)
    elseif goal_status.status == ActionLibGoalStatus.ACTIVE then
      transitionToState(self, goal, CommState.ACTIVE)
    elseif goal_status.status == ActionLibGoalStatus.PREEMPTED then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.PREEMPTING)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == ActionLibGoalStatus.SUCCEEDED then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == ActionLibGoalStatus.ABORTED then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == ActionLibGoalStatus.REJECTED then
      transitionToState(self, goal, CommState.PENDING)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == ActionLibGoalStatus.RECALLED then
      transitionToState(self, goal, CommState.PENDING)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == ActionLibGoalStatus.PREEMPTING then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.PREEMPTING)
    elseif goal_status.status == ActionLibGoalStatus.RECALLING then
      transitionToState(self, goal, CommState.PENDING)
      transitionToState(self, goal, CommState.RECALLING)
    else
      ros.ERROR_NAMED("ActionClient", "BUG: Got an unknown status from the ActionServer. status = %u", goal_status.status)
    end

  elseif goal.state == CommState.PENDING then

    if goal_status.status == ActionLibGoalStatus.PENDING then
      ; -- nop
    else if goal_status.status == ActionLibGoalStatus.ACTIVE then
      transitionToState(self, goal, CommState.ACTIVE)
    elseif goal_status.status == ActionLibGoalStatus.PREEMPTED then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.PREEMPTING)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == ActionLibGoalStatus.SUCCEEDED then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == ActionLibGoalStatus.ABORTED then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == ActionLibGoalStatus.REJECTED then
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == ActionLibGoalStatus.RECALLED then
      transitionToState(self, goal, CommState.RECALLING)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == ActionLibGoalStatus.PREEMPTING then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.PREEMPTING)
    elseif goal_status.status == ActionLibGoalStatus.RECALLING then
      transitionToState(self, goal, CommState.RECALLING)
    else
      ros.ERROR_NAMED("ActionClient", "BUG: Got an unknown status from the ActionServer. status = %u", goal_status.status)
    end

  elseif goal.state == CommState.ACTIVE then
    -- TODO

  elseif goal.state == CommState.WAITING_FOR_RESULT then
    -- TODO

  elseif goal.state == CommState.WAITING_FOR_CANCEL_ACK then
    -- TODO

  elseif goal.state == CommState.RECALLING then
    -- TODO

  elseif goal.state == CommState.PREEMPTING then

    if goal_status.status == ActionLibGoalStatus.PENDING then
      ros.ERROR_NAMED("ActionClient", "Invalid Transition from PREEMPTING to PENDING")
    elseif goal_status.status == ActionLibGoalStatus.ACTIVE then
      ros.ERROR_NAMED("ActionClient", "Invalid Transition from PREEMPTING to ACTIVE")
     elseif goal_status.status == ActionLibGoalStatus.REJECTED then
      ros.ERROR_NAMED("ActionClient", "Invalid Transition from PREEMPTING to REJECTED")
    elseif goal_status.status == ActionLibGoalStatus.RECALLING then
      ros.ERROR_NAMED("ActionClient", "Invalid Transition from PREEMPTING to RECALLING")
    elseif goal_status.status == ActionLibGoalStatus.RECALLED then
      ros.ERROR_NAMED("ActionClient", "Invalid Transition from PREEMPTING to RECALLED")
    elseif goal_status.status == ActionLibGoalStatus.PREEMPTED or
      goal_status.status == ActionLibGoalStatus.SUCCEEDED or
      goal_status.status == ActionLibGoalStatus.ABORTED then
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == ActionLibGoalStatus.PREEMPTING
      ; -- nop
    else
      ros.ERROR_NAMED("ActionClient", "BUG: Got an unknown state from the ActionServer. status = %u", goal_status.status)
    end

  elseif goal.state == CommState.DONE then

    if goal_status.status == ActionLibGoalStatus.PENDING then
      ros.ERROR_NAMED("ActionClient", "Invalid Transition from DONE to PENDING")
    elseif goal_status.status == ActionLibGoalStatus.ACTIVE then
      ros.ERROR_NAMED("ActionClient", "Invalid Transition from DONE to ACTIVE")
    elseif goal_status.status == ActionLibGoalStatus.RECALLING then
      ros.ERROR_NAMED("ActionClient", "Invalid Transition from DONE to RECALLING")
    elseif goal_status.status == ActionLibGoalStatus.PREEMPTING then
      ros.ERROR_NAMED("ActionClient", "Invalid Transition from DONE to PREEMPTING")
    elseif goal_status.status == ActionLibGoalStatus.PREEMPTED or
      goal_status.status == ActionLibGoalStatus.SUCCEEDED or
      goal_status.status == ActionLibGoalStatus.ABORTED or
      goal_status.status == ActionLibGoalStatus.RECALLED or
      goal_status.status == ActionLibGoalStatus.REJECTED) then
      ; -- nop
    else
      ros.ERROR_NAMED("ActionClient", "BUG: Got an unknown status from the ActionServer. status = %u", goal_status.status)
    end

  else
    ros.ERROR_NAMED("ActionClient", "Invalid comm state: %u", goal.state)
  end

end

local function findGoalInStatusList(status_list, goal_id)
  for i, status in ipairs(status_list)
    if status.goal_id == goal_id then
      return status
    end
  end
  return nil
end

local function onStatusMessage(self, status_msg, header)
  local callerid = header["callerid"]
  ros.DEBUG_NAMED("ActionClient", "Getting status over the wire (callerid: %s).", callerid)

  if self.status_received then
    if self.status_caller_id ~= callerid then
      ros.WARN_NAMED("ActionClient", "onStatusMessage: Previously received status from [%s], but we now received status from [%s]. Did the ActionServer change?",
                   self.status_caller_id, callerid)
      self.status_caller_id = callerid
    end
  else
    ros.DEBUG_NAMED("ActionClient", "onStatusMessage: Just got our first status message from the ActionServer at node [%s]", callerid)
    self.status_received = true
    self.status_caller_id = callerid
  end
  self.latest_status_time = status_msg.header.stamp

  -- process status message
  local status_list = status_msg.status_list

  for i, goal in self.goals do
    local goal_status = findGoalInStatusList(status_list, goal.id)
    updateStatus(self, goal, goal_status)
  end
end

local function onFeedbackMessage(self, action_feedback)
  local goal = self.goals[action_feedback.status.goal_id.id]
  if goal ~= nil and goal.feedback_cb ~= nil then
    goal.feedback_cb(goal, action_feedback.feedback)
  end
end

local function onResultMessage(self, action_result)
  local goal = self.goals[action_feedback.status.goal_id.id]
  if goal ~= nil then
    self.latest_goal_status = action_result.status
    self.latest_result = action_result

    if goal.state == CommState.DONE then
      ros.ROS_ERROR_NAMED("ActionClient", "Got a result when we were already in the DONE state")
    elseif goal.state == CommState.WAITING_FOR_GOAL_ACK or
      goal.state == CommState.PENDING or
      goal.state == CommState.ACTIVE or
      goal.state == CommState.WAITING_FOR_RESULT or
      goal.state == CommState.WAITING_FOR_CANCEL_ACK or
      goal.state == CommState.RECALLING or
      goal.state == CommState.PREEMPTING then

      updateStatus(self, goal, action_result.status)
      transitionToState(self, goal, CommState.DONE)
    else
      ros.ROS_ERROR_NAMED("ActionClient", "Invalid comm for result message state: %u.", goal.state)
    end

  end
end

function ActionClient:__init(action_spec, name, parent_node_handle, callback_queue)
  callback_queue = callback_queue or ros.DEFAULT_CALLBACK_QUEUE

  self.action_spec = action_spec
  self.nh = ros.NodeHandle(name, parent_node_handle)
  self.status_received = false
  self.goals = {}

  self.name = name
  self.goalSubscribers = {}
  self.cancelSubscribers = {}

  self.status_sub   = self.nh:subscribe("status", "actionlib_msgs/GoalStatusArray", 50)
  self.feedback_sub = self.nh:subscribe("feedback", action_spec.action_feedback_spec, 50)
  self.result_sub   = self.nh:subscribe("result", action_spec.action_result_spec, 50)

  self.status_sub:registerCallback(function(msg, header) onStatusMessage(self, msg, header) end)
  self.feedback_sub:registerCallback(function(msg) onFeedbackMessage(self, msg) end)
  self.result_sub:registerCallback(function(msg) onResultMessage(self, msg) end)

  self.goal_pub = self.nh.advertise("goal", action_spec.action_goal_spec, 10, false,
    function(name, topic) goalConnectCallback(self, name, topic) end,
    function(name, topic) goalDisconnectCallback(self, name, topic) end,
    callback_queue
  )

  self.cancel_pub = self.nh.advertise("cancel", "actionlib_msgs/GoalID", 10, false,
    function(name, topic) cancelConnectCallback(self, name, topic) end,
    function(name, topic) cancelDisconnectCallback(self, name, topic) end,
    callback_queue
  )
end

function ActionClient:shutdown()
  self.status_sub:shutdown()
  self.feedback_sub:shutdown()
  self.result_sub:shutdown()
  self.goal_pub:shutdown()
  self.cancel_pub:shutdown()
end

function ActionClient:sendGoal(goal, transition_cb, feedback_cb)
  -- create goal id
  local id_msg = ros.Message('actionlib_msgs/GoalID')
  local now = ros.Time.now()
  id_msg.id = string.format("%s-%d-%d.%d", ros.this_node.getName(), next_goal_id, now:get_sec(), now:get_nsec())
  id_msg.stap = now
  next_goal_id = next_goal_id + 1

  -- prepare goal message
  local action_goal = ros.Message(self.action_spec.action_goal_spec)
  action_goal.header.stamp = ros.Time.now()
  action_goal.goal_id = id_msg
  action_goal.goal = goal

  -- register goal in goal table
  local gh = {
    id = id_msg.id,
    action_goal = action_goal,
    state = CommState.WAITING_FOR_GOAL_ACK,
    transition_cb = transition_cb,
    feedback_cb = feedback_cb
  }
  self.goals[action_goal.goal_id] = gh

  -- publish goal message
  self.goal_pub:publish(action_goal)
  ros.DEBUG_NAMED("ActionClient", "Goal published")
  return gh
end

function ActionClient:cancelGoalsAtAndBeforeTime(time)
  local cancel_msg = ros.Message('actionlib_msgs/GoalID')
  cancel_msg.stamp = time
  self.cancel_pub:publish(cancel_msg)
end

function ActionClient:cancelAllGoals()
  self:cancelGoalsAtAndBeforeTime(ros.Time())     -- CancelAll is encoded by stamp=0
end

function ActionClient:waitForActionServerToStart(timeout)
  if timeout ~= nil and type(timeout) == 'number' then
    timeout = ros.Duration(timeout)
  end
  if timeout <= ros.Duration(0) then
    timeout = nil
  end
  local tic = ros.Time.now()
  while true do
    if self:isServerConnected() then
      return true
    end

    if timeout ~= nil
      local toc = ros.Time.now()
      if toc - tic > timeout then
        return false  -- timeout
      end
    end

    ros.spinOnce()  -- process incoming messages
  end
end

local function formatSubscriberDebugString(name, list)
  local subscribers = utils.getTableKeys(list)
  local s = name .. string.format(" (%d total)", #subscribers)
  if #subscribers > 0 then s = s .. "\n   - " end
  return s .. table.concat(subscribers, "\n   - ")
end

function ActionClient:isServerConnected()
  if not self.status_received then
    ros.DEBUG_NAMED("ActionClient", "isServerConnected: Didn't receive status yet, so not connected yet")
    return false
  end

  if self.goalSubscribers[self.status_caller_id] == nil then
    ros.DEBUG_NAMED("ActionClient", "isServerConnected: Server [%s] has not yet subscribed to the goal topic, so not connected yet", self.status_caller_id)
    ros.DEBUG_NAMED("ActionClient", "%s", formatSubscriberDebugString("goalSubscribers", self.goalSubscribers))
    return false;
  end

  if self.cancelSubscribers[self.status_caller_id] == nil then
    ros.DEBUG_NAMED("ActionClient", "isServerConnected: Server [%s] has not yet subscribed to the cancel topic, so not connected yet", self.status_caller_id)
    ros.DEBUG_NAMED("ActionClient", "%s", formatSubscriberDebugString("cancelSubscribers", self.cancelSubscribers))
    return false;
  end

  if feedback_sub:getNumPublishers() == 0 then
    ros.DEBUG_NAMED("ActionClient", "isServerConnected: Client has not yet connected to feedback topic of server [%s]", self.status_caller_id)
    return false
  end

  if result_sub:getNumPublishers() == 0 then
    ros.DEBUG_NAMED("ActionClient", "isServerConnected: Client has not yet connected to result topic of server [%s]", self.status_caller_id)
    return false
  end

  ros.DEBUG_NAMED("ActionClient", "isServerConnected: Server [%s] is fully connected.", self.status_caller_id)
  return true
end
