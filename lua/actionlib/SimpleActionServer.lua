--- SimpleActionServer
-- The SimpleActionServer implements a single goal policy on top of the
-- ActionServer class. The specification of the policy is as follows: only one
-- goal can have an active status at a time, new goals preempt previous goals
-- based on the stamp in their GoalID field (later goals preempt earlier ones),
-- an explicit preempt goal preempts all goals with timestamps that are less
-- than or equal to the stamp associated with the preempt, accepting a new goal
-- implies successful preemption of any old goal and the status of the old goal
-- will be change automatically to reflect this.
-- @classmod SimpleActionServer

local ros = require 'ros.env'
local GoalStatus = require 'ros.actionlib.GoalStatus'
require 'ros.actionlib.ActionServer'
local std = ros.std
local actionlib = ros.actionlib
local CommState = actionlib.CommState   -- CommState enum


local SimpleActionServer = torch.class('ros.actionlib.SimpleActionServer', actionlib)


-- private functions
local function SimpleActionServer_goalCallback(self, goal)
  ros.DEBUG_NAMED("actionlib", "A new goal has been recieved by the single goal action server")

  -- check that the timestamp is past or equal to that of the current goal and the next goal
  if (self.current_goal:getGoal() == nil or goal:getGoalID().stamp >= self.current_goal:getGoalID().stamp) and
      (self.next_goal == nil or self.next_goal:getGoal() == nil or goal:getGoalID().stamp >= self.next_goal:getGoalID().stamp) then

    -- if next_goal has not been accepted already... its going to get bumped, but we need to let the client know we're preempting
    if self.next_goal ~= nil and self.next_goal:getGoal() ~= nil and self.current_goal:getGoal() ~= nil then
      self.next_goal:setCanceled(nil, "This goal was canceled because another goal was recieved by the simple action server")
    end

    self.next_goal = goal
    self.new_goal = true
    self.new_goal_preempt_request = false

    -- if the server is active, we'll want to call the preempt callback for the current goal
    if self:isActive() then
      self.preempt_request = true
      -- if the user has registered a preempt callback, we'll call it now
      if self.preempt_callback ~= nil then
        self.preempt_callback(self)
      end
    end

    -- if the user has defined a goal callback, we'll call it now
    if self.goal_callback then
      self.goal_callback(self)
    end

  else
    -- the goal requested has already been preempted by a different goal, so we're not going to execute it
    goal:setCanceled(nil, "This goal was canceled because another goal was recieved by the simple action server")
  end
end


local function SimpleActionServer_preemptCallback(self, preempt)
  ros.DEBUG_NAMED("actionlib", "A preempt has been received by the SimpleActionServer")

  --if the preempt is for the current goal, then we'll set the preemptRequest flag and call the user's preempt callback
  if preempt == self.current_goal then
    ros.DEBUG_NAMED("actionlib", "Setting preempt_request bit for the current goal to TRUE and invoking callback")
    self.preempt_request = true

    -- if the user has registered a preempt callback, we'll call it now
    if self.preempt_callback then
      self.preempt_callback(self)
    end

  -- if the preempt applies to the next goal, we'll set the preempt bit for that
  elseif preempt == self.next_goal then
    ros.DEBUG_NAMED("actionlib", "Setting preempt request bit for the next goal to TRUE")
    self.new_goal_preempt_request = true
  end
end


function SimpleActionServer:__init(node_handle, name, action_spec)
  self.new_goal = false
  self.preempt_request = false
  self.new_goal_preempt_request = false
  self.need_to_terminate = false

  self.as = actionlib.ActionServer(node_handle, name, action_spec)
  self.as:registerGoalCallback(function(goal) SimpleActionServer_goalCallback(self, goal) end)
  self.as:registerCancelCallback(function(goal) SimpleActionServer_preemptCallback(self, goal) end)

  -- set initial dummy goal
  self.current_goal = actionlib.ServerGoalHandle(self.as, ros.Message('actionlib_msgs/GoalID'), GoalStatus.ABORTED, nil)
end


--- Start the action server.
function SimpleActionServer:start()
  self.as:start()
end


--- Shutdown the action server
function SimpleActionServer:shutdown()
  self.as:shutdown()
end


--- Accepts a new goal when one is available.
-- The status of this goal is set to active upon acceptance, and the
-- status of any previously active goal is set to preempted. Preempts
-- received for the new goal between checking if isNewGoalAvailable or
-- invokation of a goal callback and the acceptNewGoal call will not
-- trigger a preempt callback. This means, isPreemptReqauested should
-- be called after accepting the goal even for callback-based implementations
-- to make sure the new goal does not have a pending preempt request.
-- @return A new goal.
function SimpleActionServer:acceptNewGoal()
  if self.new_goal == false or self.next_goal == nil or self.next_goal:getGoal() == nil then
    ros.ERROR_NAMED("actionlib", "Attempting to accept the next goal when a new goal is not available")
    return nil
  end

  -- check if we need to send a preempted message for the goal that we're currently pursuing
  if self:isActive() and self.current_goal:getGoal() ~= nil then
    self.current_goal:setCanceled(nil, "This goal was canceled because another goal was recieved by the simple action server")
  end

  ros.DEBUG_NAMED("actionlib", "Accepting a new goal")

  -- accept the next goal
  self.current_goal = self.next_goal
  self.new_goal = false

  -- set preempt to request to equal the preempt state of the new goal
  self.preempt_request = self.new_goal_preempt_request
  self.new_goal_preempt_request = false

  --set the status of the current goal to be active
  self.current_goal:setAccepted("This goal has been accepted by the simple action server")

  return self.current_goal:getGoal()
end


function SimpleActionServer:getCurrentGoal()
  return self.current_goal:getGoal()
end


function SimpleActionServer:getCurrentGoalHandle()
  return self.current_goal
end


function SimpleActionServer:createResult()
  return self.as:createResult()
end


function SimpleActionServer:createFeeback()
  return self.as:createFeeback()
end


--- Allows to query about the availability of a new goal.
-- @return true if a new goal is available, false otherwise
function SimpleActionServer:isNewGoalAvailable()
  return self.new_goal
end

--- Allows to query about preempt requests.
-- @return true if a preempt is requested, false otherwise
function SimpleActionServer:isPreemptRequested()
  return self.preempt_request
end


--- Allows to query about the status of the current goal.
-- @return true if a goal is active, false otherwise
function SimpleActionServer:isActive()
  if self.current_goal == nil or self.current_goal:getGoal() == nil then
    return false
  end

  local status = self.current_goal:getGoalStatus()
  return status == GoalStatus.ACTIVE or status == GoalStatus.PREEMPTING
end


--- Sets the status of the active goal to succeeded.
-- @param result An optional result to send back to any clients of the goal
-- @param text An optional text message to send back to any clients of the goal
function SimpleActionServer:setSucceeded(result, text)
  ros.DEBUG_NAMED("actionlib", "Setting the current goal as succeeded");
  self.current_goal:setSucceeded(result, text)
end


--- Sets the status of the active goal to aborted.
-- @param result An optional result to send back to any clients of the goal
-- @param text An optional text message to send back to any clients of the goal
function SimpleActionServer:setAborted(result, text)
  ros.DEBUG_NAMED("actionlib", "Setting the current goal as aborted");
  self.current_goal:setAborted(result, text)
end


--- Publishes feedback for a given goal.
-- @param feedback Feedback object to publish
function SimpleActionServer:publishFeedback(feedback)
  self.current_goal:publishFeedback(feedback)
end


--- Sets the status of the active goal to preempted.
-- @param result An optional result to send back to any clients of the goal
-- @param text An optional text message to send back to any clients of the goal
function SimpleActionServer:setPreempted(result, text)
  ros.DEBUG_NAMED("actionlib", "Setting the current goal as canceled");
  self.current_goal:setCanceled(result, text)
end


--- Allows users to register a callback to be invoked when a new goal is available.
-- @param goal_cb The callback to be invoked
function SimpleActionServer:registerGoalCallback(goal_cb)
  self.goal_callback = goal_cb
end


--- Allows users to register a callback to be invoked when a new preempt request is available.
-- @param preempt_cb The callback to be invoked
function SimpleActionServer:registerPreemptCallback(preempt_cb)
  self.preempt_callback = preempt_cb
end
