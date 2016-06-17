--- Message buffer class
-- @classmod MessageBuffer
local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local MessageBuffer = torch.class('ros.MessageBuffer', ros)

function init()
  local MessageBuffer_method_names = {
    "new",
    "delete",
    "count",
    "clear",
    "read"
  }

  return utils.create_method_table("ros_MessageBuffer_", MessageBuffer_method_names)
end

local f = init()

--- Message buffer constructor.
-- @tparam[opt] int max_backlog Maximum buffer size, if not provided, buffer size is unlimited
-- @treturn MessageBuffer The constructed object
function MessageBuffer:__init(max_backlog)
  self.o = f.new(max_backlog or -1)
end

--- Access underlying data structure
-- @return The data structure
function MessageBuffer:cdata()
  return self.o
end

--- Get the number of messages in the buffer.
-- @treturn int Number of messages
function MessageBuffer:getCount()
  return f.count(self.o)
end

--- Clear the buffer and discard all messages
function MessageBuffer:clear()
  f.clear(self.o)
end

--- Get the next message from buffer
-- @tparam int Timeout to wait for next message if the buffer is empty
-- @tparam[opt] torch.ByteStorage result_msg If present, this object is used to store the message data, otherwise the required memory is allocated by this function
-- @tparam[opt] std.StringMap result_header If present, this object is used to store the message header, otherwise the required memory is allocated by this function
-- @treturn torch.ByteStorage result_msg The message data
-- @treturn std.StringMap The message header data
function MessageBuffer:read(timeout_milliseconds, result_msg, result_header)
  result_msg = result_msg or torch.ByteStorage()
  result_header = result_header or std.StringMap()
  if not f.read(self.o, timeout_milliseconds or 100, result_msg:cdata(), result_header:cdata()) then
    return nil, nil
  end
  return result_msg, result_header
end
