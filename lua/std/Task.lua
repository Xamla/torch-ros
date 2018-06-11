local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local Task = torch.class('std.Task', std)

local TaskState = {
  NotStarted = 1,
  Running = 2,
  Succeeded = 3,
  Failed = 4,
  Cancelled = 5,
}
std.TaskState = TaskState


function Task.create(start_handler, cancel_handler, completion_handler, auto_start)
  local task = Task.new(start_handler, cancel_handler)

  if completion_handler ~= nil then
    task:addCompletionHandler(completion_handler)
  end

  if auto_start == true then
    task:start()
  end

  return task
end


function Task:__init(start_handler, cancel_handler)
  self.start_handler = start_handler
  self.cancel_handler = cancel_handler
  self.state = TaskState.NotStarted
  self.completion_handlers = {}
end


function Task:getState()
  return self.state
end


function Task:addCompletionHandler(completion_handler)
  self.completion_handlers[#self.completion_handlers + 1] = completion_handler
end


function Task:removeCompletionHandler(completion_handler)
  local i = utils.indexOf(self.completion_handlers, completion_handler)
  if i ~= -1 then
    table.remove(self.completion_handlers, i)
  end
end


function Task:start()
  self.state = TaskState.Running
  self:start_handler()
end


function Task:cancel(reason)
  if self.cancel_handler ~= nil then
    self:cancel_handler(reason)
  end
end


function Task:hasCompleted()
  return self.state == TaskState.Succeeded or self.state == TaskState.Failed or self.state == TaskState.Cancelled
end


function Task:hasCompletedSuccessfully()
  return self.state == TaskState.Succeeded
end


function Task:waitForCompletion(timeout_in_ms, spin_rate, spin_function)
  spin_rate = spin_rate or 25
  if not torch.isTypeOf(spin_rate, ros.Rate) then
    spin_rate = ros.Rate(spin_rate)
  end

  spin_function = spin_function or ros.spinOnce
  local start_time = ros.Time.now()

  while not self:hasCompleted() do
    if timeout_in_ms ~= nil and (ros.Time.now() - start_time):toSec() > timeout_in_ms / 1000 then
      return false
    end
    spin_rate:sleep()
    spin_function()
  end

  return true
end


function Task:getResult()
  if not self:hasCompleted() then
    self:waitForCompletion()
  end

  return self.result
end


function Task:setResult(terminal_state, value)
  if self:hasCompleted() then
    error('Task already completed.')
  end

  self.state = terminal_state
  self.result = value
  assert(self:hasCompleted(), 'Invalid terminal state specified.')

  for i, handler in ipairs(self.completion_handlers) do
    handler(self)
  end
end
