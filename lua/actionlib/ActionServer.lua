local ros = require 'ros.env'
local std = ros.std

local actionlib = ros.actionlib

--[[
http://wiki.ros.org/actionlib
http://wiki.ros.org/actionlib/DetailedDescription
https://github.com/ros/actionlib/tree/indigo-devel/include/actionlib
https://github.com/timn/ros-actionlib_lua/blob/master/src/actionlib/action_client.lua
]]


local ActionServer = torch.class('ros.actionlib.ActionServer', actionlib)

local GoalID_spec = ros.get_msgspec('actionlib_msgs/GoalID ')
local GoalStatusArray_spec = ros.get_msgspec('actionlib_msgs/GoalStatusArray')


function goalCallback(self, goal)

end

function cancelCallback(self, goal_id)

end


local function initialize(self)
  -- msg emitting topics
  self.result_pub   = self.node:advertise("result", self.action_spec.action_result_spec, 50)
  self.feedback_pub = self.node:advertise("feedback", self.action_spec.action_feedback_spec, 50)
  self.status_pub   = self.node:advertise("status", GoalStatusArray_spec, 50)

  self.status_list = {}

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
    -- ## TODO: add timer support
    -- self.status_timer = self.node:createTimer(ros.Duration(1.0 / status_frequency), function() publishStatus(self))
  end

  -- msg absorbing topics
  self.goal_sub     = self.node:subscribe("goal", self.action_spec.action_goal_spec, 50)
  self.cancel_sub   = self.node:subscribe("cancel", GoalID_spec, 50)
end

function ActionServer:__init(node_handle, action_name, action_spec)
  self.node = node_handle
  self.action_spec = action_spec
  self.started = false
end

function ActionServer:start()
  self.started = true
  initialize(self)
  self:publishStatus()
end

function ActionServer:publishResult(status, result)
  local ar = self.result_pub:createMessage()
  ar.header.stamp = ros.Time.now()
  ar.status = status
  ar.result = result
  ros.DEBUG_NAMED("actionlib", "Publishing result for goal with id: %s and stamp: %.2f", status.goal_id.id, status.goal_id.stamp:toSec())
  self.result_pub:publish(ar)
  self:publishStatus()
end

function ActionServer:publishFeedback(status, feedback)
  local af = self.feedback_pub:createMessage()
  af.header.stamp = ros.Time.now()
  af.status = status
  af.feedback = feedback
  ros.DEBUG_NAMED("actionlib", "Publishing feedback for goal with id: %s and stamp: %.2f", status.goal_id.id, status.goal_id:stamp.toSec())
  self.feedback_pub:publish(af)
end

function ActionServer:publishStatus()
  -- build a status array
  local status_array = ros.Message(GoalStatusArray_spec)
  status_array.header.stamp = ros.Time.now()

  local i = 1
  while i < #self.status_list do
    local s = self.status_list[i]
    table.insert(status_array.status_list, s.status)    -- add status message to list

    -- check if the item is due for deletion from the status list
    if s.handle_destruction_time ~= nil and s.handle_destruction_time:add(self.status_list_timeout) < ros.Time.now() then
      table.remove(self.status_list, i)   -- remove item
    else
      i = i + 1
    end
  end

  self.status_pub:publish(status_array)
end
