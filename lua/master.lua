--- Collection of functions to query information about the ROS master
-- @module master
local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local master = {}
ros.master = master

function init()
  local names = {
    'execute',
    'getHost',
    'getPort',
    'getURI',
    'check',
    'getTopics',
    'getNodes',
    'setRetryTimeout'
  }

  return utils.create_method_table("ros_Master_", names)
end

local f = init()

--- Execute an XMLRPC call on the master.
-- @tparam string method The RPC method to invoke
-- @tparam std.Variable request	The arguments to the RPC call
-- @tparam bool wait_for_master Whether or not this call should loop until it can contact the master
-- @treturn bool result true if the master is available, false otherwise.
-- @treturn std.Variable response The resonse that was received.
-- @treturn std.Variable payload The payload that was received.
function master.execute(method, request, wait_for_master)
  if not torch.isTypeOf(request, std.Variable) then
    request = std.Variable(request)
  end
  local response, payload = std.Variable(), std.Variable()
  local result = f.execute(method, request:cdata(), response:cdata(), payload:cdata(), wait_for_master or false)
  return result, response:get(), payload:get()
end

--- Get the hostname where the master runs.
-- @treturn string The master's hostname
function master.getHost()
  return ffi.string(f.getHost())
end

--- Get the port where the master runs.
-- @treturn string The master's port.
function master.getPort()
  return f.getPort()
end

--- Get the full URI to the master (eg. http://host:port/).
-- @treturn string The URI of the master
function master.getURI()
  return ffi.string(f.getURI())
end

--- Check if the master is running.
-- This method tries to contact the master. You can call it any time after ros::init has been called.
-- The intended usage is to check whether the master is up before trying to make other
-- requests (subscriptions, advertisements, etc.).
-- @treturn bool true if the master is available, false otherwise.
function master.check()
  return f.check()
end

--- Get the list of topics that are being published by all nodes.
-- @tparam[opt] std.VariableTable output Will be filled the the topic names. If parameter is not present, the table will be created by the function
-- @treturn std.VariableTable Table containing the names of all topics
function master.getTopics(output)
  local v = output or std.VariableTable()
  f.getTopics(v:cdata())
  return v:totable()
end

--- Retreives the currently-known list of nodes from the master.
-- @tparam[opt] std.StringVector output Will be filled with the list of nodes. If parameter is not present, the object will be created by this function
-- @treturn std.StringVector List of nodes
function master.getNodes(output)
  local v = output or std.StringVector()
  f.getNodes(v:cdata())
  return v
end

--- Set the max time this node should spend looping trying to connect to the master.
-- @tparam ?number|ros.Duration _1 If number: time duration in secons, fractional number possible
-- @tparam[opt] number _2 If present, _1 represends the seconds and _2 represends the nanoseconds
function master.setRetryTimeout(_1, _2)
  local d = _1
  if not torch.isTypeOf(d, ros.Duration) then
    d = ros.Duration(_1, _2)
  end
  f.setRetryTimeout(d:get_sec(), d:get_nsec())
end
