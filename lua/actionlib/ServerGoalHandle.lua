local ros = require 'ros.env'
local GoalStatus = require 'ros.actionlib.GoalStatus'
local std = ros.std
local actionlib = ros.actionlib


local ServerGoalHandle = torch.class('ros.actionlib.ServerGoalHandle', actionlib)


function setGoalStatus(self, status, text)
  self.goal_status.status = status
  self.goal_status.text = text
  self.action_server:publishStatus()
end


function setGoalResult(self, status, text, result)
  self.goal_status.status = status
  self.goal_status.text = text
  self.action_server:publishResult(self.goal_status, result)
  self.handle_destruction_time = ros.Time.now()
end


function ServerGoalHandle:__init(action_server, goal_id, status, goal)
  self.action_server = action_server
  self.goal_status = ros.Message('actionlib_msgs/GoalStatus')     -- http://docs.ros.org/jade/api/actionlib_msgs/html/msg/GoalStatus.html
  self.goal_status.goal_id = goal_id

  if self.goal_status.goal_id.stamp == ros.Time() then
   self.goal_status.goal_id.stamp = ros.Time.now()
  end

  self.goal_status.status = status
  self.goal = goal
end


function ServerGoalHandle:createResult()
  return self.action_server:createResult()
end


function ServerGoalHandle:createFeeback()
  return self.action_server:createFeeback()
end


function ServerGoalHandle:setAccepted(text)
  text = text or ''
  ros.DEBUG_NAMED("actionlib", "Accepting goal, id: %s, stamp: %.2f", self:getGoalID().id, self:getGoalID().stamp:toSec())
  if self.goal_status.status == GoalStatus.PENDING then
    setGoalStatus(self, GoalStatus.ACTIVE, text)
  elseif self.goal_status.status == GoalStatus.RECALLING then
    setGoalStatus(self, GoalStatus.PREEMPTING, text)
  else
    ros.ERROR_NAMED("actionlib", "To transition to an active state, the goal must be in a pending or recalling state, it is currently in state: %d",
      self.goal_status.status)
  end
end


function ServerGoalHandle:setCanceled(result, text)
  text = text or ''
  ros.DEBUG_NAMED("actionlib", "Setting status to canceled on goal, id: %s, stamp: %.2f", self:getGoalID().id, self:getGoalID().stamp:toSec())
  if self.goal_status.status == GoalStatus.PENDING or self.goal_status.status == GoalStatus.RECALLING then
    setGoalResult(self, GoalStatus.RECALLED, text, result)
  elseif self.goal_status.status == GoalStatus.ACTIVE or self.goal_status.status == GoalStatus.PREEMPTING then
    setGoalResult(self, GoalStatus.PREEMPTED, text, result)
  else
    ros.ERROR_NAMED("actionlib", "To transition to a cancelled state, the goal must be in a pending, recalling, active, or preempting state, it is currently in state: %d",
      self.goal_status.status)
  end
end


function ServerGoalHandle:setRejected(result, text)
  text = text or ''
  ros.DEBUG_NAMED("actionlib", "Setting status to rejected on goal, id: %s, stamp: %.2f", self:getGoalID().id, self:getGoalID().stamp:toSec())
  if self.goal_status.status == GoalStatus.PENDING or self.goal_status.status == GoalStatus.RECALLING then
    setGoalResult(self, GoalStatus.REJECTED, text, result)
  else
    ros.ERROR_NAMED("actionlib", "To transition to a rejected state, the goal must be in a pending or recalling state, it is currently in state: %d",
      self.goal_status.status)
  end
end


function ServerGoalHandle:setAborted(result, text)
  text = text or ''
  ros.DEBUG_NAMED("actionlib", "Setting status to aborted on goal, id: %s, stamp: %.2f", self:getGoalID().id, self:getGoalID().stamp:toSec())
  if self.goal_status.status == GoalStatus.PREEMPTING or self.goal_status.status == GoalStatus.ACTIVE then
    setGoalResult(self, GoalStatus.ABORTED, text, result)
  else
    ros.ERROR_NAMED("actionlib", "To transition to an aborted state, the goal must be in a preempting or active state, it is currently in state: %d",
      self.goal_status.status)
  end
end


function ServerGoalHandle:setSucceeded(result, text)
  text = text or ''
  ros.DEBUG_NAMED("actionlib", "Setting status to succeeded on goal, id: %s, stamp: %.2f", self:getGoalID().id, self:getGoalID().stamp:toSec())
  if self.goal_status.status == GoalStatus.PREEMPTING or self.goal_status.status == GoalStatus.ACTIVE then
    setGoalResult(self, GoalStatus.SUCCEEDED, text, result)
  else
    ros.ERROR_NAMED("actionlib", "To transition to a succeeded state, the goal must be in a preempting or active state, it is currently in state: %d",
      self.goal_status.status)
  end
end


function ServerGoalHandle:publishFeedback(feedback)
  self.action_server:publishFeedback(self.goal_status, feedback)
end


function ServerGoalHandle:getGoal()
  return self.goal
end


function ServerGoalHandle:getGoalID()
  return self.goal_status.goal_id
end


function ServerGoalHandle:getGoalStatus()
  return self.goal_status
end


function ServerGoalHandle:setCancelRequested()
  ros.DEBUG_NAMED("actionlib", "Transisitoning to a cancel requested state on goal id: %s, stamp: %.2f", self:getGoalID().id, self:getGoalID().stamp:toSec())
  if self.goal_status.status == GoalStatus.PENDING then
    setGoalStatus(self, GoalStatus.RECALLING)
    return true
  end
  if self.goal_status.status == GoalStatus.ACTIVE then
    setGoalStatus(self, GoalStatus.PREEMPTING)
    return true
  end
  return false
end
