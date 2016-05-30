local ros = require 'ros.env'
require 'ros.ros'
require 'ros.Time'
require 'ros.Duration'
require 'ros.console'
require 'ros.StorageWriter'
require 'ros.StorageReader'
require 'ros.MsgSpec'
require 'ros.Message'
require 'ros.NodeHandle'
local GoalStatus = require 'ros.actionlib.GoalStatus'
require 'ros.actionlib.ServerGoalHandle'
local std = ros.std
local actionlib = ros.actionlib

--[[
http://wiki.ros.org/actionlib
http://wiki.ros.org/actionlib/DetailedDescription
https://github.com/ros/actionlib/tree/indigo-devel/include/actionlib
]]

--- actionlib_msgs/GoalID Message
-- time stamp
-- string id
local GoalID_spec = ros.get_msgspec('actionlib_msgs/GoalID')                      -- http://docs.ros.org/jade/api/actionlib_msgs/html/msg/GoalID.html

--- actionlib_msgs/GoalStatusArray Message
-- std_msgs/Header header
-- actionlib_msgs/GoalStatus[] status_list
local GoalStatusArray_spec = ros.get_msgspec('actionlib_msgs/GoalStatusArray')    -- http://docs.ros.org/jade/api/actionlib_msgs/html/msg/GoalStatusArray.html


local ActionServer = torch.class('ros.actionlib.ActionServer', actionlib)


local function goalCallback(self, goal)
  if not self.started then return end

  ros.DEBUG_NAMED("actionlib", "The action server has received a new goal request")

  local id = goal.goal_id.id
  local gh = self.status_list[id]
  if gh ~= nil then
    -- The goal could already be in a recalling state if a cancel came in before the goal
    if gh.goal_status.status == GoalStatus.RECALLING then
      gh.goal_status.status = GoalStatus.RECALLED
      self:publishResult(gh.goal_status, nil)  -- empty result
    end
  else
    -- create and register new goal handle
    gh = actionlib.ServerGoalHandle(self, goal.goal_id, GoalStatus.PENDING, goal)
    self.status_list[id] = gh
    if self.goal_callback then
      self.goal_callback(gh)
    end
  end
end


local function cancelCallback(self, goal_id)
  if not self.started then return end

  ros.DEBUG_NAMED("actionlib", "The action server has received a new cancel request")

  local id = goal_id.id
  if #id == 0 then   -- empty goal id
    local time_zero = ros.Time()
    for k,v in pairs(self.status_list) do
      if goal_id.stamp == time_zero or v.goal_status.goal_id.stamp < goal_id.stamp then
        if v:setCancelRequested() and self.cancel_callback then
          self.cancel_callback(v)
        end
      end
    end
  else
    local gh = self.status_list[id]
    if gh ~= nil then
      -- entry found, cancel
      if gh:setCancelRequested() and self.cancel_callback then
        self.cancel_callback(gh)
      end
    else
      -- we have not received the goal yet, prepare to cancel goal when it is received
      gh = actionlib.ServerGoalHandle(self, goal_id, GoalStatus.RECALLING)
      gh.handle_destruction_time = goal_id.stamp   -- start the timer for how long the status will live in the list without a goal handle to it
      self.status_list[id] = gh
    end
  end

  -- make sure to set last_cancel based on the stamp associated with this cancel request
  if goal_id.stamp > self.last_cancel then
    self.last_cancel = goal_id.stamp
  end
end


function ActionServer:__init(node_handle, action_name, action_spec, callback_queue)
  if type(action_spec) == 'string' then
    action_spec = actionlib.ActionSpec(action_spec)
  end

  callback_queue = callback_queue or ros.DEFAULT_CALLBACK_QUEUE
  self.callback_queue = callback_queue
  self.status_list = {}
  self.node = ros.NodeHandle(action_name, node_handle)
  self.action_spec = action_spec
  self.last_cancel = ros.Time.now()
  self.started = false
end


function ActionServer:shutdown()
  if self.spin_cb ~= nil then
    ros.unregisterSpinCallback(self.spin_cb)
    self.spin_cb = nil
  end
  self.result_pub:shutdown()
  self.feedback_pub:shutdown()
  self.status_pub:shutdown()
  self.goal_sub:shutdown()
  self.cancel_sub:shutdown()
end


function ActionServer:createResult()
  return ros.Message(self.action_spec.result_spec)
end


function ActionServer:createFeeback()
  return ros.Message(self.action_spec.feedback_spec)
end


function ActionServer:start()
  if self.started then
    return
  end

  -- msg emitting topics
  self.result_pub   = self.node:advertise("result", self.action_spec.action_result_spec, 50)
  self.feedback_pub = self.node:advertise("feedback", self.action_spec.action_feedback_spec, 50)
  self.status_pub   = self.node:advertise("status", GoalStatusArray_spec, 50)

  -- read the frequency with which to publish status from the parameter server
  -- if not specified locally explicitly, use search param to find actionlib_status_frequency
  local status_frequency, ok = self.node:getParamDouble("actionlib_status_frequency")
  if not ok then
    status_frequency = 5.0
  end

  local status_list_timeout, ok = self.node:getParamDouble("status_list_timeout")
  if not ok then
    status_list_timeout = 5.0
  end

  self.status_list_timeout = ros.Duration(status_list_timeout);

  -- start timer
  if status_frequency > 0 then
    self.status_interval = ros.Duration(1.0 / status_frequency)
    self.next_status_publish_time = ros.Time.now() + self.status_interval
    self.spin_cb = function()
      if ros.Time.now() > self.next_status_publish_time then
        self:publishStatus()
        self.next_status_publish_time = ros.Time.now() + self.status_interval
      end
    end
    ros.registerSpinCallback(self.spin_cb)
  end

  -- msg absorbing topics
  self.goal_sub     = self.node:subscribe("goal", self.action_spec.action_goal_spec, 50, nil, nil, callback_queue)
  self.cancel_sub   = self.node:subscribe("cancel", GoalID_spec, 50, nil, nil, callback_queue)

  self.started = true

  self.goal_sub:registerCallback(function(msg) goalCallback(self, msg) end)
  self.cancel_sub:registerCallback(function(msg) cancelCallback(self, msg) end)

  self:publishStatus()
end


function ActionServer:registerGoalCallback(goal_cb)
  self.goal_callback = goal_cb
end


function ActionServer:registerCancelCallback(cancel_cb)
  self.cancel_callback = cancel_cb
end


function ActionServer:publishResult(status, result)
  local action_result = self.result_pub:createMessage()
  action_result.header.stamp = ros.Time.now()
  action_result.status = status
  if result ~= nil then
    action_result.result = result
  end
  ros.DEBUG_NAMED("actionlib", "Publishing result for goal with id: %s and stamp: %.2f", status.goal_id.id, status.goal_id.stamp:toSec())
  self.result_pub:publish(action_result)
  self:publishStatus()
end


function ActionServer:publishFeedback(status, feedback)
  local acfion_feedback = self.feedback_pub:createMessage()
  acfion_feedback.header.stamp = ros.Time.now()
  acfion_feedback.status = status
  acfion_feedback.feedback = feedback
  ros.DEBUG_NAMED("actionlib", "Publishing feedback for goal with id: %s and stamp: %.2f", status.goal_id.id, status.goal_id.stamp:toSec())
  self.feedback_pub:publish(acfion_feedback)
end


function ActionServer:publishStatus()
  -- fill a status array message and publish it
  local now = ros.Time.now()
  local status_array = self.status_pub:createMessage()
  status_array.header.stamp = now

  local i = 1
  for k,v in pairs(self.status_list) do
    table.insert(status_array.status_list, v.status)    -- add status message to list
    -- check if the item is due for deletion from the status list
    if v.handle_destruction_time ~= nil and v.handle_destruction_time:add(self.status_list_timeout) < now then
      self.status_list[k] = nil   -- remove item
    end
  end

  self.status_pub:publish(status_array)
end
