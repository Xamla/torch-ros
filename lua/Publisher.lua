--- Manages an advertisement on a specific topic.
-- A Publisher should always be created through a call to
-- NodeHandle::advertise(), or copied from one that was. Once all
-- copies of a specific Publisher go out of scope, any subscriber
-- status callbacks associated with that handle will stop being
-- called. Once all Publishers for a given topic go out of scope the
-- topic will be unadvertised.
-- @classmod Publisher
local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local tf = ros.tf

local Publisher = torch.class('ros.Publisher', ros)
local Publisher_ptr_ct = ffi.typeof('ros_Publisher *')

function init()
  local Publisher_method_names = {
    'clone',
    'delete',
    'shutdown',
    'getTopic',
    'getNumSubscribers',
    'isLatched',
    'publish'
  }

  return utils.create_method_table('ros_Publisher_', Publisher_method_names)
end

local f = init()

--- Constructor.
-- A Publisher should always be created through a call to
-- NodeHandle::advertise(), or copied from one that was. Therefore parameters are not documented in detail
-- @param ptr
-- @param msg_spec
-- @param connect_cb
-- @param disconnect_cb
function Publisher:__init(ptr, msg_spec, connect_cb, disconnect_cb, serialization_handlers)
  if not ptr or not ffi.typeof(ptr) == Publisher_ptr_ct then
    error('argument 1: ros::Publisher * expected.')
  end
  self.o = ptr
  self.msg_spec = msg_spec
  self.connect_cb = connect_cb
  self.disconnect_cb = disconnect_cb
  self.serialization_handlers = serialization_handlers

  ffi.gc(ptr,
    function(p)
      f.delete(p)
      if self.connect_cb ~= nil then
        self.connect_cb:free()      -- free connect callback
        self.connect_cb = nil
      end
      if self.disconnect_cb ~= nil then
        self.disconnect_cb:free()   -- free disconnet callback
        self.disconnect_cb = nil
      end
    end
  )

end

--- Get the cdata of this object
function Publisher:cdata()
  return self.o
end

--- Create a deep copy
-- @treturn ros.Publisher The new object
function Publisher:clone()
  local c = torch.factory('ros.Publisher')()
  rawset(c, 'o', f.clone(self.o))
  rawset(c, 'msg_spec', self.msg_spec)
  return c
end

--- Shutdown the publisher.
-- This method usually does not need to be explicitly called, as
-- automatic shutdown happens when all copies of this Publisher go out
-- of scope
function Publisher:shutdown()
  f.shutdown(self.o)
end

--- Returns the topic that this Publisher will publish on.
-- @treturn string The topic that this Publisher will publish on.
function Publisher:getTopic()
  return ffi.string(f.getTopic(self.o))
end

--- Number of subscribers of this publisher.
-- @treturn int Number of subscribers
function Publisher:getNumSubscribers()
  return f.getNumSubscribers(self.o)
end

--- Returns whether or not this topic is latched.
-- @treturn bool whether or not this topic is latched
function Publisher:isLatched()
  return f.isLatched(self.o)
end

--- Publish the given message
-- @tparam ros.Message msg The message to publish
function Publisher:publish(msg)
  local sw = ros.StorageWriter(nil, 0, self.serialization_handlers)
  if torch.isTypeOf(msg, ros.Message) then
    -- serialize message to byte storage
    msg:serialize(sw)

  else
    -- get serialization handler by message type
    local handler = sw:getHandler(self.msg_spec.type)
    if handler == nil then
      error('No serialization handler defined for msg type')
    end

    sw:writeUInt32(0)   -- reserve space for message size
    handler:write(sw, msg)
    sw:writeUInt32(sw.offset - 4, offset)
  end

  sw:shrinkToFit()
  f.publish(self.o, sw.storage:cdata(), 0, sw.length)
end

--- Wait for subscribers
-- @tparam int min_count Minimum numbers of subscribers to wait for
-- @tparam ?ros.Duration|number Maximum number of seconds to wait for subscribers
-- @treturn bool true if the number of subscribers is reached, false if timed out
function Publisher:waitForSubscriber(min_count, timeout)
  if not ros.Time.isValid() then
    ros.Time.init()
  end

  min_count = min_count or 1
  if timeout and not torch.isTypeOf(timeout, ros.Duration) then
    timeout = ros.Duration(timeout)
  end

  local start = ros.Time.getNow()
  while true do
    if timeout and (ros.Time.getNow() - start) > timeout then
      return false
    elseif self:getNumSubscribers() >= min_count then
      return true
    end
    ros.spinOnce()
    sys.sleep(0.001)
  end
end

---
function Publisher:createMessage()
  return ros.Message(self.msg_spec)
end
