local ros = require 'ros.env'
require 'ros.actionlib.ActionServer'
local std = ros.std
local actionlib = ros.actionlib

local SimpleActionServer = torch.class('ros.actionlib.SimpleActionServer', actionlib)

local CommState = actionlib.CommState

-- private functions
local function goalCallback(self, goal)
end

local function preemptCallback(self, preempt)
end


function SimpleActionServer:__init(node_handle, name, execute_callback)
  self.new_goal = false
  self.preempt_request = false
  self.new_goal_preempt_request = false
  self.execute_callback = execute_callback
  self.need_to_terminate = false
end


function SimpleActionServer:start()
end


function SimpleActionServer:shutdown()
end


function SimpleActionServer:acceptNewGoal()
end


function SimpleActionServer:isNewGoalAvailable()
end


function SimpleActionServer:isPreemptRequested()
end


function SimpleActionServer:isActive()
end


function SimpleActionServer:setSucceeded(result, text)
end


function SimpleActionServer:setAborted(result, text)
end


function SimpleActionServer:publishFeedback(feedback)
end


function SimpleActionServer:setPreempted()
end


function SimpleActionServer:registerGoalCallback()
end


function SimpleActionServer:registerPreemptCallback()
end
