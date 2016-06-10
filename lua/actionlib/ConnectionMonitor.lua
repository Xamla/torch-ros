local ros = require 'ros.env'
local std = ros.std
local actionlib = ros.actionlib


-- internal debug tracing helper functions
local function CONNECTION_DEBUG(...)
  ros.DEBUG_NAMED("ConnectionMonitor", ...)
end


local function CONNECTION_WARN(fmt, ...)
  ros.WARN_NAMED("ConnectionMonitor", ...)
end


local ConnectionMonitor = torch.class('ros.actionlib.ConnectionMonitor', actionlib)


local function cancelSubscribersString(self)
  local ss = {}
  table.insert(ss, string.format("Cancel Subscribers (%d total)", self.cancelSubscribersCount))
  for k,v in pairs(self.cancelSubscribers) do
    table.insert(ss, k)
  return table.concat(ss, '\n')
end


local function goalSubscribersString(self)
  local ss = {}
  table.insert(ss, string.format("Goal Subscribers (%d total)", self.goalSubscribersCount))
  for k,v in pairs(self.goalSubscribers) do
    table.insert(ss, k)
  return table.concat(ss, '\n')
end


function ConnectionMonitor:__init(feedback_sub, result_sub)
  self.feedback_sub = feedback_sub
  self.result_sub = result_sub
  self.status_received = false
  self.status_caller_id = ''
  self.cancelSubscribers = {}
  self.cancelSubscribersCount = 0
  self.goalSubscribers = {}
  self.goalSubscribersCount = 0
end


function ConnectionMonitor:goalConnectCallback(subscriber_name)
  if self.goalSubscribers[subscriber_name] == nil then      -- Check if it's not in the list
    CONNECTION_DEBUG("goalConnectCallback: Adding [%s] to goalSubscribers", subscriber_name)
    self.goalSubscribers[subscriber_name] = 1
    self.goalSubscribersCount = self.goalSubscribersCount + 1
  else
    CONNECTION_WARN("goalConnectCallback: Trying to add [%s] to goalSubscribers, but it is already in the goalSubscribers list", subscriber_name)
    self.goalSubscribers[subscriber_name] = self.goalSubscribers[subscriber_name] + 1
  end
  CONNECTION_DEBUG("%s", goalSubscribersString(self))
end


function ConnectionMonitor:goalDisconnectCallback(subscriber_name)
  if self.goalSubscribers[subscriber_name] == nil then
    CONNECTION_WARN("goalDisconnectCallback: Trying to remove [%s] to goalSubscribers, but it is not in the goalSubscribers list", subscriber_name)
  else
    CONNECTION_DEBUG("goalDisconnectCallback: Removing [%s] from goalSubscribers", subscriber_name)
    self.goalSubscribers[subscriber_name] = self.goalSubscribers[subscriber_name] - 1
    if self.goalSubscribers[subscriber_name] <= 0 then
      self.goalSubscribers[subscriber_name] = nil
      self.goalSubscribersCount = self.goalSubscribersCount - 1
    end
  end
  CONNECTION_DEBUG("%s", goalSubscribersString(self))
end


function ConnectionMonitor:cancelConnectCallback(subscriber_name)
  if self.cancelSubscribers[subscriber_name] == nil then     -- Check if it's not in the list
    CONNECTION_DEBUG("cancelConnectCallback: Adding [%s] to cancelSubscribers", subscriber_name)
    self.cancelSubscribers[subscriber_name] = 1
    self.cancelSubscribersCount = self.cancelSubscribersCount + 1
  else
    CONNECTION_WARN("cancelConnectCallback: Trying to add [%s] to cancelSubscribers, but it is already in the cancelSubscribers list", subscriber_name)
    self.cancelSubscribers[subscriber_name] = self.cancelSubscribers[subscriber_name] + 1
  end
  CONNECTION_DEBUG("%s", cancelSubscribersString(self))
end


function ConnectionMonitor:cancelDisconnectCallback(subscriber_name)
  if self.cancelSubscribers[subscriber_name] == nil then
    CONNECTION_WARN("cancelDisconnectCallback: Trying to remove [%s] to cancelSubscribers, but it is not in the cancelSubscribers list", subscriber_name)
  else
    CONNECTION_DEBUG("cancelDisconnectCallback: Removing [%s] from cancelSubscribers", subscriber_name)
    self.cancelSubscribers[subscriber_name] = self.cancelSubscribers[subscriber_name] - 1
    if self.cancelSubscribers[subscriber_name] <= 0 then
      self.cancelSubscribers[subscriber_name] = nil
      self.cancelSubscribersCount = self.cancelSubscribersCount - 1
    end
  }
  CONNECTION_DEBUG("%s", cancelSubscribersString(self))
end


function ConnectionMonitor:processStatus(status_msg, caller_id)
  if self.status_received then
    if self.status_caller_id != caller_id then
      CONNECTION_WARN(
        "processStatus: Previously received status from [%s], but we now received status from [%s]. Did the ActionServer change?",
        self.status_caller_id, caller_id
      )
      self.status_caller_id = caller_id
    end
    self.latest_status_time = status_msg.header.stamp
  else
    CONNECTION_DEBUG("processStatus: Just got our first status message from the ActionServer at node [%s]", caller_id)
    self.status_received = true
    self.status_caller_id = caller_id
    self.latest_status_time = status_msg.header.stamp
  end
end


function ConnectionMonitor:waitForActionServerToStart(timeout, node_handle )
  timeout = timeout or ros.Duration(0, 0)
  node_handle = node_handle or ros.NodeHandle()

  if timeout < ros.Duration(0) then
    ros.ERROR_NAMED("actionlib", "Timeouts can't be negative. Timeout is [%.2fs]", timeout:toSec())
  end

  local timeout_time = ros.Time.now() + timeout

  if self:isServerConnected() then
    return true
  end

  while self.nh:ok() and not self.isServerConnected() do
    -- Determine how long we should wait
    local time_left = timeout_time - ros.Time.now()

    -- Check if we're past the timeout time
    if timeout ~= ros.Duration(0) and time_left <= ros.Duration(0) then
      break
    end

    ros.spinOnce()
    sys.sleep(0.01)   -- check 100 times a second
  end

  return self:isServerConnected()
end


function ConnectionMonitor:isServerConnected()
  if not self.status_received then
    CONNECTION_DEBUG("isServerConnected: Didn't receive status yet, so not connected yet");
    return false
  end

  if self.goalSubscribers[self.status_caller_id] == nil then
    CONNECTION_DEBUG("isServerConnected: Server [%s] has not yet subscribed to the goal topic, so not connected yet", self.status_caller_id);
    CONNECTION_DEBUG("%s", cancelSubscribersString(self))
    return false
  end

  if self.cancelSubscribers[self.status_caller_id] == nil then
    CONNECTION_DEBUG("isServerConnected: Server [%s] has not yet subscribed to the cancel topic, so not connected yet", self.status_caller_id);
    CONNECTION_DEBUG("%s", cancelSubscribersString(self))
    return false
  end

  if self.feedback_sub:getNumPublishers() == 0 then
    CONNECTION_DEBUG("isServerConnected: Client has not yet connected to feedback topic of server [%s]", status_caller_id)
    return false
  end

  if self.result_sub:getNumPublishers() == 0 then
    CONNECTION_DEBUG("isServerConnected: Client has not yet connected to result topic of server [%s]", status_caller_id)
    return false
  end

  CONNECTION_DEBUG("isServerConnected: Server [%s] is fully connected", status_caller_id)
  return true
end
