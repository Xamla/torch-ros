--- SerializedMessage class
-- @classmod SerializedMessage
local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local SerializedMessage = torch.class('ros.SerializedMessage', ros)
local SerializedMessage_ptr_ct = ffi.typeof('ros_SerializedMessage *')

function init()
  local SerializedMessage_method_names = {
    "new",
    "delete",
    "view",
    "size",
    "data",
    "resize"
  }

  return utils.create_method_table("ros_SerializedMessage_", SerializedMessage_method_names)
end

local f = init()

--- SerializedMessage constructor.
function SerializedMessage:__init()
  self.o = f.new()
end

--- internal function do no use in normal client code
function SerializedMessage.fromPtr(ptr)
  if not ffi.istype(SerializedMessage_ptr_ct, ptr) then
    error('ros::SerializedMessage* expected.')
  end
  local c = torch.factory('ros.SerializedMessage')()
  rawset(c, 'o', ptr)
  return c
end

--- Access underlying data structure
-- @return The data structure
function SerializedMessage:cdata()
  return self.o
end

--- Get the number of bytes in the buffer.
-- @treturn int Number of bytes
function SerializedMessage:size()
  return f.size(self.o)
end

--- Creates a tensor object directly looking into the internal memory buffer of the SerializeMessage object without copying.
-- @return Tensor with storage pointing to the internal buffer.
function SerializedMessage:view(output_tensor)
  output_tensor = output_tensor or torch.ByteTensor()
  f.view(self.o, output_tensor:cdata())
  return output_tensor
end

function SerializedMessage:data()
  return f.data(self.o)
end

function SerializedMessage:resize(new_size)
  return f.resize(self.o, new_size)
end
