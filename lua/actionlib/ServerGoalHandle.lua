local ros = require 'ros.env'
local GoalStatus = require 'ros.actionlib.GoalStatus'
local std = ros.std
local actionlib = ros.actionlib


local ServerGoalHandle = torch.class('ros.actionlib.ServerGoalHandle', actionlib)


local function ServerGoalHandle_setGoalStatus(self, status, text)
  self.goal_status.status = status
  self.goal_status.text = text or ''
  self.action_server:publishStatus()
end


local function ServerGoalHandle_setGoalResult(self, status, text, result)
  self.goal_status.status = status
  self.goal_status.text = text or ''
  self.action_server:publishResult(self.goal_status, result)
  self.handle_destruction_time = ros.Time.now()
end


function ServerGoalHandle:__init(action_server, goal_id, status, goal)
  self.action_server = action_server
  self.goal_status = ros.Message('actionlib_msgs/GoalStatus')     -- http://docs.ros.org/jade/api/actionlib_msgs/html/msg/GoalStatus.html
  self.goal_status.goal_id:assign(goal_id)

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
    ServerGoalHandle_setGoalStatus(self, GoalStatus.ACTIVE, text)
  elseif self.goal_status.status == GoalStatus.RECALLING then
    ServerGoalHandle_setGoalStatus(self, GoalStatus.PREEMPTING, text)
  else
    ros.ERROR_NAMED("actionlib", "To transition to an active state, the goal must be in a pending or recalling state, it is currently in state: %d",
      self.goal_status.status)
  end
end


function ServerGoalHandle:setCanceled(result, text)
  text = text or ''
  ros.DEBUG_NAMED("actionlib", "Setting status to canceled on goal, id: %s, stamp: %.2f", self:getGoalID().id, self:getGoalID().stamp:toSec())
  if self.goal_status.status == GoalStatus.PENDING or self.goal_status.status == GoalStatus.RECALLING then
    ServerGoalHandle_setGoalResult(self, GoalStatus.RECALLED, text, result)
  elseif self.goal_status.status == GoalStatus.ACTIVE or self.goal_status.status == GoalStatus.PREEMPTING then
    ServerGoalHandle_setGoalResult(self, GoalStatus.PREEMPTED, text, result)
  else
    errer('blub')
    ros.ERROR_NAMED("actionlib", "To transition to a cancelled state, the goal must be in a pending, recalling, active, or preempting state, it is currently in state: %d",
      self.goal_status.status)
  end
end


function ServerGoalHandle:setRejected(result, text)
  text = text or ''
  ros.DEBUG_NAMED("actionlib", "Setting status to rejected on goal, id: %s, stamp: %.2f", self:getGoalID().id, self:getGoalID().stamp:toSec())
  if self.goal_status.status == GoalStatus.PENDING or self.goal_status.status == GoalStatus.RECALLING then
    ServerGoalHandle_setGoalResult(self, GoalStatus.REJECTED, text, result)
  else
    ros.ERROR_NAMED("actionlib", "To transition to a rejected state, the goal must be in a pending or recalling state, it is currently in state: %d",
      self.goal_status.status)
  end
end


function ServerGoalHandle:setAborted(result, text)
  text = text or ''
  ros.DEBUG_NAMED("actionlib", "Setting status to aborted on goal, id: %s, stamp: %.2f", self:getGoalID().id, self:getGoalID().stamp:toSec())
  if self.goal_status.status == GoalStatus.PREEMPTING or self.goal_status.status == GoalStatus.ACTIVE then
    ServerGoalHandle_setGoalResult(self, GoalStatus.ABORTED, text, result)
  else
    ros.ERROR_NAMED("actionlib", "To transition to an aborted state, the goal must be in a preempting or active state, it is currently in state: %d",
      self.goal_status.status)
  end
end


function ServerGoalHandle:setSucceeded(result, text)
  text = text or ''
  ros.DEBUG_NAMED("actionlib", "Setting status to succeeded on goal, id: %s, stamp: %.2f", self:getGoalID().id, self:getGoalID().stamp:toSec())
  if self.goal_status.status == GoalStatus.PREEMPTING or self.goal_status.status == GoalStatus.ACTIVE then
    ServerGoalHandle_setGoalResult(self, GoalStatus.SUCCEEDED, text, result)
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
    ServerGoalHandle_setGoalStatus(self, GoalStatus.RECALLING, 'RECALLING')
    return true
  end
  if self.goal_status.status == GoalStatus.ACTIVE then
    ServerGoalHandle_setGoalStatus(self, GoalStatus.PREEMPTING, 'PREEMPTING')
    return true
  end
  return false
end
