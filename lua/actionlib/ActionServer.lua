local ros = require 'ros.env'
local std = ros.std

--[[
http://wiki.ros.org/actionlib
http://wiki.ros.org/actionlib/DetailedDescription
https://github.com/ros/actionlib/tree/indigo-devel/include/actionlib
https://github.com/timn/ros-actionlib_lua/blob/master/src/actionlib/action_client.lua
]]

if ros.actionlib == nil then ros.actionlib = {} end

local actionlib = ros.actionlib

local ActionServer = torch.class('ros.actionlib.ActionServer', actionlib)

local GoalID_spec = ros.get_msgspec('actionlib_msgs/GoalID ')
local GoalStatusArray_spec = ros.get_msgspec('actionlib_msgs/GoalStatusArray')

local function initialize(self)
  --self.result_pub = self.node:advertise<ActionResult>("result", , 50)
  --self.feedback_pub = self.node:advertise<ActionFeedback>("feedback", , 50)
  self.status_pub = self.node:advertise("status", GoalStatusArray_spec, 50)

  --self.goal_sub = self.node:subscribe<ActionGoal>("goal", , 50)
  self.cancel_sub = self.node:subscribe("cancel", GoalID_spec, 50)
end

function ActionServer:__init(node_handle, action_name, action_spec)
  self.nh = node

end
