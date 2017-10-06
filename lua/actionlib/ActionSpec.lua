local md5 = require 'md5'
local path = require 'pl.path'
local torch = require 'torch'
local ros = require 'ros.env'
local actionlib = ros.actionlib


local ActionSpec = torch.class('ros.actionlib.ActionSpec', actionlib)
local DEFAULT_PACKAGE = 'actionlib'


--- (internal) load from iterator
-- @param iterator iterator that returns one line of the specification at a time
local function load_from_iterator(self, iterator)
  local goal, result, feedback  = {}, {}, {}

  -- extract goal, result and feedback messages from action descriptions
  local t = goal
  for line in iterator do
    if string.find(line, '^%s*---%s*$') ~= nil then
      if t == goal then
        t = result
      else
        t = feedback
      end
    else
      table.insert(t, line)
    end
  end

  -- generate inner structures
  self.goal_spec  = ros.get_msgspec(self.type .. 'Goal', table.concat(goal, '\n'))
  self.result_spec = ros.get_msgspec(self.type .. 'Result', table.concat(result, '\n'))
  self.feedback_spec = ros.get_msgspec(self.type .. 'Feedback', table.concat(feedback, '\n'))

  -- generate derived
  local action_goal_msg = 'Header header\nactionlib_msgs/GoalID goal_id\n' .. self.type .. 'Goal goal'
  local action_result_msg = 'Header header\nactionlib_msgs/GoalStatus status\n' .. self.type .. 'Result result'
  local action_feedback_msg = 'Header header\nactionlib_msgs/GoalStatus status\n' .. self.type .. 'Feedback feedback'

  self.action_goal_spec  = ros.get_msgspec(self.type .. 'ActionGoal', action_goal_msg)
  self.action_result_spec = ros.get_msgspec(self.type .. 'ActionResult', action_result_msg)
  self.action_feedback_spec = ros.get_msgspec(self.type .. 'ActionFeedback', action_feedback_msg)
end


local function load_from_action_file(self)
  local package_path = ros.find_package(self.package)
  local tmp_path = path.join(package_path, 'action')
  self.file = path.join(tmp_path, self.short_type .. '.action')
  return load_from_iterator(self, io.lines(self.file))
end


--- (internal) Load specification from string.
-- @param s string containing the message specification
local function load_from_string(self, s)
  return load_from_iterator(self, s:gmatch('([^\r\n]+)\n?'))
end


function ActionSpec:__init(type, specstr)
  assert(type, 'Action type is expected')
  self.type = type

  local slashpos = type:find('/')
  if slashpos then
    self.package    = type:sub(1, slashpos - 1)
    self.short_type = type:sub(slashpos + 1)
  else
    self.package    = DEFAULT_PACKAGE
    self.short_type = type
  end

  if specstr then
    load_from_string(self, specstr)
  else
    load_from_action_file(self)
  end
end


function ActionSpec:format_spec(ln)
  table.insert(ln, 'Action ' .. self.type)
  self.action_goal_spec:format_spec(ln)
  table.insert(ln, '---')
  self.action_result_spec:format_spec(ln)
  table.insert(ln, '---')
  self.action_feedback_spec:format_spec(ln)
  return ln
end


function ActionSpec:__tostring()
  lines = self:format_spec({})
  table.insert(lines, '')
  return table.concat(lines, '\n')
end
