local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local ServiceServer = torch.class('ros.ServiceServer', ros)
local ServiceServer_ptr_ct = ffi.typeof('ros_ServiceServer *')

function init()
  local ServiceServer_method_names = {
    "delete",
    "shutdown",
    "getService"
  }

  return utils.create_method_table("ros_ServiceServer_", ServiceServer_method_names)
end

local f = init()

function ServiceServer:__init(ptr, callback, service_handler_func)
  if not ffi.istype(ServiceServer_ptr_ct, ptr) then
    error('ros::ServiceServer* expected.')
  end

  self.o = ptr
  self.callback = callback
  self.handler = service_handler_func
  ffi.gc(ptr,
    function(p)
      f.delete(p)
      if self.callback ~= nil then
        self.callback:free()      -- free callback
        self.callback = nil
      end
    end
  )

end

function ServiceServer:cdata()
  return self.o
end

function ServiceServer:shutdown()
  return f.shutdown(self.o)
end

function ServiceServer:getService()
  local result = std.String()
  f.getService(self.o, result:cdata())
  return result:get()
end
