local ros = require 'ros.env'
local utils = require 'ros.utils'
local GoalStatus = require 'ros.actionlib.GoalStatus'
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
actionlib.CommState = CommState


local next_goal_id = 1    -- shared among all action clients
local ActionClient = torch.class('ros.actionlib.ActionClient', actionlib)


local function goalConnectCallback(self, name, topic)
  self.goalSubscribers[name] = (self.goalSubscribers[name] or 0) + 1
  ros.DEBUG_NAMED("actionlib", "goalConnectCallback: Adding [%s] to goalSubscribers", name)
end


local function goalDisconnectCallback(self, name, topic)
  local count = self.goalSubscribers[name]
  if count == nil then
    -- should never happen, warning copied from official actionlib impl
    ros.WARN_NAMED("actionlib", "goalDisconnectCallback: Trying to remove [%s] from goalSubscribers, but it is not in the goalSubscribers list.", name)
  else
    ros.DEBUG_NAMED("actionlib", "goalDisconnectCallback: Removing [%s] from goalSubscribers, (remaining with same name: %d)", name, count - 1)
    if count <= 1 then
      self.goalSubscribers[name] = nil
    else
      self.goalSubscribers[name] = count - 1
    end
  end
end


local function cancelConnectCallback(self, name, topic)
  self.cancelSubscribers[name] = (self.cancelSubscribers[name] or 0) + 1
  ros.DEBUG_NAMED("actionlib", "cancelConnectCallback: Adding [%s] to cancelSubscribers", name)
end


local function cancelDisconnectCallback(self, name, topic)
  local count = self.cancelSubscribers[name]
  if count == nil then
    -- should never happen, warning copied from official actionlib impl
    ros.WARN_NAMED("actionlib", "cancelDisconnectCallback: Trying to remove [%s] from cancelSubscribers, but it is not in the cancelSubscribers list.", name)
  else
    ros.DEBUG_NAMED("actionlib", "cancelDisconnectCallback: Removing [%s] from cancelSubscribers (remaining with same name: %d)", name, count - 1)
    if count <= 1 then
      self.cancelSubscribers[name] = nil
    else
      self.cancelSubscribers[name] = count - 1
    end
  end
end


local function transitionToState(self, goal, next_state)
  ros.DEBUG_NAMED("actionlib", "Transitioning CommState from %s to %s", CommState[goal.state], CommState[next_state])
  goal.state = next_state
  if goal.transition_cb ~= nil then
    goal.transition_cb(goal)
  end
end


local function processLost(self, goal)
  ros.WARN_NAMED("actionlib", "Transitioning goal to LOST")
  if goal.latest_goal_status ~= nil then
    goal.latest_goal_status.status = GoalStatus.LOST -- LOST
    goal.latest_goal_status.text = "LOST"
  end
  transitionToState(self, goal, CommState.DONE)
end


local function updateStatus(self, goal, goal_status)
  -- check if pending action is correctly reflected by the status message
  if goal_status ~= nil then
    goal.latest_goal_status = goal_status
  else
    if goal.state ~= CommState.WAITING_FOR_GOAL_ACK and goal.state ~= CommState.WAITING_FOR_RESULT and goal.state ~= CommState.DONE then
      processLost(self, goal)
    end
    return
  end

  if goal.state == CommState.WAITING_FOR_GOAL_ACK then

    if goal_status.status == GoalStatus.PENDING then
      transitionToState(self, goal, CommState.PENDING)
    elseif goal_status.status == GoalStatus.ACTIVE then
      transitionToState(self, goal, CommState.ACTIVE)
    elseif goal_status.status == GoalStatus.PREEMPTED then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.PREEMPTING)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.SUCCEEDED then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.ABORTED then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.REJECTED then
      transitionToState(self, goal, CommState.PENDING)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.RECALLED then
      transitionToState(self, goal, CommState.PENDING)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.PREEMPTING then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.PREEMPTING)
    elseif goal_status.status == GoalStatus.RECALLING then
      transitionToState(self, goal, CommState.PENDING)
      transitionToState(self, goal, CommState.RECALLING)
    else
      ros.ERROR_NAMED("actionlib", "BUG: Got an unknown status from the ActionServer. status = %u", goal_status.status)
    end

  elseif goal.state == CommState.PENDING then

    if goal_status.status == GoalStatus.PENDING then
      ; -- nop
    elseif goal_status.status == GoalStatus.ACTIVE then
      transitionToState(self, goal, CommState.ACTIVE)
    elseif goal_status.status == GoalStatus.PREEMPTED then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.PREEMPTING)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.SUCCEEDED then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.ABORTED then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.REJECTED then
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.RECALLED then
      transitionToState(self, goal, CommState.RECALLING)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.PREEMPTING then
      transitionToState(self, goal, CommState.ACTIVE)
      transitionToState(self, goal, CommState.PREEMPTING)
    elseif goal_status.status == GoalStatus.RECALLING then
      transitionToState(self, goal, CommState.RECALLING)
    else
      ros.ERROR_NAMED("actionlib", "BUG: Got an unknown status from the ActionServer. status = %u", goal_status.status)
    end

  elseif goal.state == CommState.ACTIVE then

    if goal_status.status == GoalStatus.PENDING then
      ros.ERROR_NAMED("actionlib", "Invalid transition from ACTIVE to PENDING")
    elseif goal_status.status == GoalStatus.ACTIVE then
      ; -- nop
    elseif goal_status.status == GoalStatus.REJECTED then
      ros.ERROR_NAMED("actionlib", "Invalid transition from ACTIVE to REJECTED")
    elseif goal_status.status == GoalStatus.RECALLING then
      ros.ERROR_NAMED("actionlib", "Invalid transition from ACTIVE to RECALLING")
    elseif goal_status.status == GoalStatus.RECALLED then
      ros.ERROR_NAMED("actionlib", "Invalid transition from ACTIVE to RECALLED")
    elseif goal_status.status == GoalStatus.PREEMPTED then
      transitionToState(self, goal, CommState.PREEMPTING)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.SUCCEEDED or
        goal_status.status == GoalStatus.ABORTED then
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.PREEMPTING then
      transitionToState(self, goal, CommState.PREEMPTING)
    else
      ros.ERROR_NAMED("actionlib", "BUG: Got an unknown status from the ActionServer. status = %u", goal_status.status)
    end

  elseif goal.state == CommState.WAITING_FOR_RESULT then

    if goal_status.status == GoalStatus.PENDING then
      ros.ERROR_NAMED("actionlib", "Invalid Transition from WAITING_FOR_RESUT to PENDING")
    elseif goal_status.status == GoalStatus.PREEMPTING then
      ros.ERROR_NAMED("actionlib", "Invalid transition from WAITING_FOR_RESUT to PREEMPTING")
    elseif goal_status.status == GoalStatus.RECALLING then
      ros.ERROR_NAMED("actionlib", "Invalid transition from WAITING_FOR_RESUT to RECALLING")
    elseif goal_status.status == GoalStatus.ACTIVE or
        goal_status.status == GoalStatus.PREEMPTED or
        goal_status.status == GoalStatus.SUCCEEDED or
        goal_status.status == GoalStatus.ABORTED or
        goal_status.status == GoalStatus.REJECTED or
        goal_status.status == GoalStatus.RECALLED then
      ; -- nop
    else
      ros.ERROR_NAMED("actionlib", "BUG: Got an unknown status from the ActionServer. status = %u", goal_status.status)
    end

  elseif goal.state == CommState.WAITING_FOR_CANCEL_ACK then

    if goal_status.status == GoalStatus.PENDING or
        goal_status.status == GoalStatus.ACTIVE then
      ; -- nop
    elseif goal_status.status == GoalStatus.SUCCEEDED or
        goal_status.status == GoalStatus.ABORTED or
        goal_status.status == GoalStatus.PREEMPTED then
      transitionToState(self, goal, CommState.PREEMPTING)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.RECALLED then
      transitionToState(self, goal, CommState.RECALLING)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.REJECTED then
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.PREEMPTING then
      transitionToState(self, goal, CommState.PREEMPTING)
    elseif goal_status.status == GoalStatus.RECALLING then
      transitionToState(self, goal, CommState.RECALLING)
    else
      ros.ERROR_NAMED("actionlib", "BUG: Got an unknown state from the ActionServer. status = %u", goal_status.status)
    end

  elseif goal.state == CommState.RECALLING then

    if goal_status.status == GoalStatus.PENDING then
      ros.ERROR_NAMED("actionlib", "Invalid Transition from RECALLING to PENDING")
    elseif goal_status.status == GoalStatus.ACTIVE then
      ros.ERROR_NAMED("actionlib", "Invalid Transition from RECALLING to ACTIVE")
    elseif goal_status.status == GoalStatus.SUCCEEDED or
        goal_status.status == GoalStatus.ABORTED or
        goal_status.status == GoalStatus.PREEMPTED then
      transitionToState(self, goal, CommState.PREEMPTING)
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.RECALLED then
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.REJECTED then
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.PREEMPTING then
      transitionToState(self, goal, CommState.PREEMPTING)
    elseif goal_status.status == GoalStatus.RECALLING then
      ; -- nop
    else
      ros.ERROR_NAMED("actionlib", "BUG: Got an unknown state from the ActionServer. status = %u", goal_status.status)
    end

  elseif goal.state == CommState.PREEMPTING then

    if goal_status.status == GoalStatus.PENDING then
      ros.ERROR_NAMED("actionlib", "Invalid Transition from PREEMPTING to PENDING")
    elseif goal_status.status == GoalStatus.ACTIVE then
      ros.ERROR_NAMED("actionlib", "Invalid Transition from PREEMPTING to ACTIVE")
     elseif goal_status.status == GoalStatus.REJECTED then
      ros.ERROR_NAMED("actionlib", "Invalid Transition from PREEMPTING to REJECTED")
    elseif goal_status.status == GoalStatus.RECALLING then
      ros.ERROR_NAMED("actionlib", "Invalid Transition from PREEMPTING to RECALLING")
    elseif goal_status.status == GoalStatus.RECALLED then
      ros.ERROR_NAMED("actionlib", "Invalid Transition from PREEMPTING to RECALLED")
    elseif goal_status.status == GoalStatus.PREEMPTED or
        goal_status.status == GoalStatus.SUCCEEDED or
        goal_status.status == GoalStatus.ABORTED then
      transitionToState(self, goal, CommState.WAITING_FOR_RESULT)
    elseif goal_status.status == GoalStatus.PREEMPTING then
      ; -- nop
    else
      ros.ERROR_NAMED("actionlib", "BUG: Got an unknown state from the ActionServer. status = %u", goal_status.status)
    end

  elseif goal.state == CommState.DONE then

    if goal_status.status == GoalStatus.PENDING then
      ros.ERROR_NAMED("actionlib", "Invalid Transition from DONE to PENDING")
    elseif goal_status.status == GoalStatus.ACTIVE then
      ros.ERROR_NAMED("actionlib", "Invalid Transition from DONE to ACTIVE")
    elseif goal_status.status == GoalStatus.RECALLING then
      ros.ERROR_NAMED("actionlib", "Invalid Transition from DONE to RECALLING")
    elseif goal_status.status == GoalStatus.PREEMPTING then
      ros.ERROR_NAMED("actionlib", "Invalid Transition from DONE to PREEMPTING")
    elseif goal_status.status == GoalStatus.PREEMPTED or
        goal_status.status == GoalStatus.SUCCEEDED or
        goal_status.status == GoalStatus.ABORTED or
        goal_status.status == GoalStatus.RECALLED or
        goal_status.status == GoalStatus.REJECTED then
      ; -- nop
    else
      ros.ERROR_NAMED("actionlib", "BUG: Got an unknown status from the ActionServer. status = %u", goal_status.status)
    end

  else
    ros.ERROR_NAMED("actionlib", "Invalid comm state: %u", goal.state)
  end

end


local function findGoalInStatusList(status_list, goal_id)
  for i, status in ipairs(status_list) do
    if status.goal_id.id == goal_id then
      return status
    end
  end
  return nil
end


local function onStatusMessage(self, status_msg, header)
  local callerid = header["callerid"]
  ros.DEBUG_NAMED("actionlib", "Getting status over the wire (callerid: %s).", callerid)

  if self.status_received then
    if self.status_caller_id ~= callerid then
      ros.WARN_NAMED("actionlib", "onStatusMessage: Previously received status from [%s], but we now received status from [%s]. Did the ActionServer change?",
                   self.status_caller_id, callerid)
      self.status_caller_id = callerid
    end
  else
    ros.DEBUG_NAMED("actionlib", "onStatusMessage: Just got our first status message from the ActionServer at node [%s]", callerid)
    self.status_received = true
    self.status_caller_id = callerid
  end
  self.latest_status_time = status_msg.header.stamp

  -- process status message
  local status_list = status_msg.status_list

  for id, goal in pairs(self.goals) do
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
    goal.latest_goal_status = action_result.status
    goal.latest_result = action_result

    if goal.state == CommState.DONE then
      ros.ROS_ERROR_NAMED("actionlib", "Got a result when we were already in the DONE state")
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
      ros.ROS_ERROR_NAMED("actionlib", "Invalid comm for result message state: %u.", goal.state)
    end

  end
end


function ActionClient:__init(action_spec, name, parent_node_handle, callback_queue)
  if type(action_spec) == 'string' then
    action_spec = actionlib.ActionSpec(action_spec)
  end

  callback_queue = callback_queue or ros.DEFAULT_CALLBACK_QUEUE
  self.callback_queue = callback_queue

  self.action_spec = action_spec
  self.nh = ros.NodeHandle(name, parent_node_handle)    -- parent_node_handle is optional, it can be nil
  self.status_received = false
  self.goals = {}

  self.name = name
  self.goalSubscribers = {}
  self.cancelSubscribers = {}

  self.status_sub   = self.nh:subscribe("status", "actionlib_msgs/GoalStatusArray", 50, nil, nil, callback_queue)
  self.feedback_sub = self.nh:subscribe("feedback", action_spec.action_feedback_spec, 50, nil, nil, callback_queue)
  self.result_sub   = self.nh:subscribe("result", action_spec.action_result_spec, 50, nil, nil, callback_queue)

  self.status_sub:registerCallback(function(msg, header) onStatusMessage(self, msg, header) end)
  self.feedback_sub:registerCallback(function(msg) onFeedbackMessage(self, msg) end)
  self.result_sub:registerCallback(function(msg) onResultMessage(self, msg) end)

  self.goal_pub = self.nh:advertise("goal", action_spec.action_goal_spec, 10, false,
    function(name, topic) goalConnectCallback(self, name, topic) end,
    function(name, topic) goalDisconnectCallback(self, name, topic) end,
    callback_queue
  )

  self.cancel_pub = self.nh:advertise("cancel", "actionlib_msgs/GoalID", 10, false,
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


--- Creates a goal message that can be filled and then sent via the sendGoal() method.
function ActionClient:createGoal()
  return ros.Message(self.action_spec.goal_spec)
end


function ActionClient:sendGoal(goal, transition_cb, feedback_cb)
  -- create goal id
  local id_msg = ros.Message('actionlib_msgs/GoalID')
  local now = ros.Time.now()
  id_msg.id = string.format("%s-%d-%d.%d", ros.this_node.getName(), next_goal_id, now:get_sec(), now:get_nsec())
  id_msg.stamp = now
  next_goal_id = next_goal_id + 1

  -- prepare goal message
  local action_goal = ros.Message(self.action_spec.action_goal_spec)
  action_goal.header.stamp = ros.Time.now()
  action_goal.goal_id = id_msg
  action_goal.goal = goal

  -- register goal in goal table
  local ac = self
  local goal_handle = {
    id = id_msg.id,
    action_goal = action_goal,
    state = CommState.WAITING_FOR_GOAL_ACK,
    transition_cb = transition_cb,
    feedback_cb = feedback_cb,
    active = true,
    reset = function(self)
      self.transition_cb = nil
      self.feedback_cb = nil
      self.active = false
    end,
    cancel = function(self)
      if not self.active then
        ros.ERROR_NAMED("actionlib", "Trying to cancel() on an inactive goal handle.")
      end

      -- check goal handle state
      if self.state == CommState.WAITING_FOR_RESULT or
          self.state == CommState.RECALLING or
          self.state == CommState.PREEMPTING or
          self.state == CommState.DONE then
        ros.DEBUG_NAMED("actionlib", "Got a cancel() request while in state [%s], so ignoring it", CommState[self.state])
        return
      elseif not (self.state == CommState.WAITING_FOR_GOAL_ACK or
          self.state == CommState.PENDING or
          self.state == CommState.ACTIVE or
          self.state == CommState.WAITING_FOR_CANCEL_ACK) then
        ros.DEBUG_NAMED("actionlib", "BUG: Unhandled CommState: %u", self.state)
        return
      end

      local cancel_msg = ros.Message('actionlib_msgs/GoalID')
      cancel_msg.id = self.id
      ac.cancel_pub:publish(cancel_msg)
      transitionToState(ac, self, CommState.WAITING_FOR_CANCEL_ACK)
    end,
    resend = function(self)
      if not self.active then
        ros.ERROR_NAMED("actionlib", "Trying to resend() on an inactive goal handle.")
      end
      ac.goal_pub:publish(self.action_goal)
    end,
    getResult = function(self)
      if not self.active then
        ros.ERROR_NAMED("actionlib", "Trying to getResult on an inactive ClientGoalHandle.")
      end
      if self.latest_result ~= nil then
        return self.latest_result.result
      end
      return nil
    end
  }
  self.goals[action_goal.goal_id.id] = goal_handle

  -- publish goal message
  self.goal_pub:publish(action_goal)
  ros.DEBUG_NAMED("actionlib", "Goal published")
  return goal_handle
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
  while ros.ok() do

    if self:isServerConnected() then
      return true
    end

    if timeout ~= nil then
      local toc = ros.Time.now()
      if toc - tic > timeout then
        return false  -- timeout
      end
    end

    ros.spinOnce()  -- process incoming messages
    sys.sleep(0.001)
  end
  return false
end


local function formatSubscriberDebugString(name, list)
  local subscribers = utils.getTableKeys(list)
  local s = name .. string.format(" (%d total)", #subscribers)
  if #subscribers > 0 then s = s .. "\n   - " end
  return s .. table.concat(subscribers, "\n   - ")
end


function ActionClient:isServerConnected()
  if not self.status_received then
    ros.DEBUG_NAMED("actionlib", "isServerConnected: Didn't receive status yet, so not connected yet")
    return false
  end

  if self.goalSubscribers[self.status_caller_id] == nil then
    ros.DEBUG_NAMED("actionlib", "isServerConnected: Server [%s] has not yet subscribed to the goal topic, so not connected yet", self.status_caller_id)
    ros.DEBUG_NAMED("actionlib", "%s", formatSubscriberDebugString("goalSubscribers", self.goalSubscribers))
    return false;
  end

  if self.cancelSubscribers[self.status_caller_id] == nil then
    ros.DEBUG_NAMED("actionlib", "isServerConnected: Server [%s] has not yet subscribed to the cancel topic, so not connected yet", self.status_caller_id)
    ros.DEBUG_NAMED("actionlib", "%s", formatSubscriberDebugString("cancelSubscribers", self.cancelSubscribers))
    return false;
  end

  if self.feedback_sub:getNumPublishers() == 0 then
    ros.DEBUG_NAMED("actionlib", "isServerConnected: Client has not yet connected to feedback topic of server [%s]", self.status_caller_id)
    return false
  end

  if self.result_sub:getNumPublishers() == 0 then
    ros.DEBUG_NAMED("actionlib", "isServerConnected: Client has not yet connected to result topic of server [%s]", self.status_caller_id)
    return false
  end

  ros.DEBUG_NAMED("actionlib", "isServerConnected: Server [%s] is fully connected.", self.status_caller_id)
  return true
end
