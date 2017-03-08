--- Interface class to create subscribers, publishers, etc.
-- @classmod NodeHandle
local ffi = require 'ffi'
local torch = require 'torch'
local ros = require 'ros.env'
local utils = require 'ros.utils'
local std = ros.std

local NodeHandle = torch.class('ros.NodeHandle', ros)

function init()
  local NodeHandle_method_names = {
    'new',
    'delete',
    'shutdown',
    'ok',
    'getNamespace',
    'getUnresolvedNamespace',
    'resolveName',
    'subscribe',
    'advertise',
    'serviceClient',
    'advertiseService',
    'hasParam',
    'deleteParam',
    'getParamString',
    'getParamDouble',
    'getParamFloat',
    'getParamInt',
    'getParamBool',
    'setParamString',
    'setParamDouble',
    'setParamFloat',
    'setParamInt',
    'setParamBool',
    'getParamStringVector',
    'getParamBoolVector',
    'getParamIntVector',
    'getParamDoubleVector',
    'getParamFloatVector',
    'setParamStringVector',
    'setParamBoolVector',
    'setParamIntVector',
    'setParamDoubleVector',
    'setParamFloatVector'
  }

  return utils.create_method_table('ros_NodeHandle_', NodeHandle_method_names)
end

local f = init()

--- NodeHandle constructor.
-- When a NodeHandle is constructed, it checks to see if the global
-- node state has already been started. If so, it increments a global
-- reference count. If not, it starts the node with ros::start() and
-- sets the reference count to 1.
-- @tparam[opt] string ns Namespace for this NodeHandle. This acts in addition to any namespace assigned to this ROS node. eg. If the node's namespace is "/a" and the namespace passed in here is "b", all topics/services/parameters will be prefixed with "/a/b/"
-- @tparam[opt] parent ros:NodeHandle Parent node handle. If the passed "ns" is relative (does not start with a slash), it is equivalent to calling: NodeHandle child(parent.getNamespace() + "/" + ns, remappings). If the passed "ns" is absolute (does start with a slash), it is equivalent to calling: NodeHandle child(ns, remappings)
-- @tparam[opt] tab remappings remapping table
function NodeHandle:__init(ns, parent, remappings)
  if remappings ~= nil and type(remappings) == 'table' then
    remappings = std.StringMap(remappings)
  end
  self.o = f.new(ns or '', utils.cdata(parent), utils.cdata(remappings))
end

--- Get the cdata of this object
function NodeHandle:cdata()
  return self.o
end

function NodeHandle:addSerializationHandler(handler)
  if self.serialization_handlers == nil then
    self.serialization_handlers = {}
  end
  self.serialization_handlers[handler:getType()] = handler
end

--- Shutdown every handle created through this NodeHandle.
-- This method will unadvertise every topic and service advertisement,
-- and unsubscribe every subscription created through this NodeHandle.
function NodeHandle:shutdown()
  f.shutdown(self.o)
end

--- Check whether it's time to exit.
-- @treturn bool true if we're still OK, false if it's time to exit
function NodeHandle:ok()
  return f.ok(self.o)
end

--- Returns the namespace associated with this NodeHandle.
-- @treturn string The namespace
function NodeHandle:getNamespace()
  return ffi.string(f.getNamespace(self.o))
end

--- Returns the namespace associated with this NodeHandle as it was passed in (before it was resolved)
-- @treturn string The unresolved namespace string
function NodeHandle:getUnresolvedNamespace()
  return ffi.string(f.getUnresolvedNamespace(self.o))
end

--- Resolves a name into a fully-qualified name.
-- @tparam string name Name to remap
-- @tparam bool remap Whether to apply name-remapping rules
-- @treturn string Resolved name
function NodeHandle:resolveName(name, remap)
  local result = std.String()
  f.resolveName(self.o, name, remap or true, result:cdata())
  return result
end

--- Subscribe to a ROS topic
-- @tparam string Topic to subscribe to
-- @tparam ?string|ros.MsgSpec msg_spec Message specification of the topic one wants to subscribe to
-- @tparam[opt=1000] int queue_size Number of incoming messages to queue up for processing (messages in excess of this queue capacity will be discarded).
-- @tparam[opt] std.StringVector transports Transport method (udp, tcp)
-- @tparam[opt] std.StringMap transport_options Various transport options like tcp_nodelay, maxDatagramSize, etc.
-- @tparam[opt] ros.CallbackQueue callback queue.
-- @treturn ros.Subscriber A ros.Subscriber object
function NodeHandle:subscribe(topic, msg_spec, queue_size, transports, transport_options, callback_queue)
  if type(msg_spec) == 'string' then
    msg_spec = ros.get_msgspec(msg_spec)
  end

  if transports ~= nil and not torch.isTypeOf(transports, std.StringVector) then
    if type(transports) == 'table' or type(transports) == 'string' then
      transports = std.StringVector(transports)
    else
      error("Invalid argument 'transports'")
    end
  end

  if transport_options ~= nil and not torch.isTypeOf(transport_options, std.StringMap) then
    if type(transport_options) == 'string' then
      local name = transport_options
      transport_options = std.StringMap(transport_options)
      transport_options[name] = 'true'
    elseif type(transport_options) == 'table' then
      transport_options = std.StringMap(transport_options)
    else
      error("Invalid argument 'transport_options'")
    end
  end

  if callback_queue ~= nil and not torch.isTypeOf(callback_queue, ros.CallbackQueue) then
    error('Invalid type of explicitly specified callback queue.')
  end

  local buffer = ros.MessageBuffer(queue_size)
  local s = f.subscribe(
    self.o,
    buffer:cdata(),
    topic,
    queue_size or 1000,
    msg_spec:md5(),
    msg_spec.type,
    utils.cdata(transports),
    utils.cdata(transport_options),
    utils.cdata(callback_queue)
  )
  return ros.Subscriber(s, buffer, msg_spec, callback_queue, self.serialization_handlers)
end

--- Advertise a topic
-- This call connects to the master to publicize that the node will be publishing messages on the given topic.
-- @tparam string topic Topic to advertise on
-- @tparam ?string|ros.MsgSpec msg_spec Message specification of the topic one wants to publish to
-- @tparam[opt=1000] int queue_size Maximum number of outgoing messages to be queued for delivery to subscribers
-- @tparam[opt=false] bool latch  If true, the last message published on this topic will be saved and sent to new subscribers when they connect
-- @tparam[opt] func  connect_cb Callback to be called if a client connects
-- @tparam[opt] func  disconnect_cb Callback to be called if a client disconnects
-- @tparam[opt=ros.DEFAULT_CALLBACK_QUEUE] ros.CallbackQueue callback_queue Callback queue
-- @treturn ros.Publisher ROS publisher
function NodeHandle:advertise(topic, msg_spec, queue_size, latch, connect_cb, disconnect_cb, callback_queue)
  if type(msg_spec) == 'string' then
    msg_spec = ros.get_msgspec(msg_spec)
  end

  if connect_cb ~= nil or disconnect_cb ~= nil then
    callback_queue = callback_queue or ros.DEFAULT_CALLBACK_QUEUE
    if not torch.isTypeOf(callback_queue, ros.CallbackQueue) then
      error('Invalid type of explicitly specified callback queue.')
    end
  else
    callback_queue = ffi.NULL
  end

  local connect_, disconnect_ = ffi.NULL, ffi.NULL
  if connect_cb ~= nil then
    connect_ = ffi.cast('_ServiceStatusCallback', function(name, topic) connect_cb(ffi.string(name), ffi.string(topic)) end)
  end
  if disconnect_cb ~= nil then
    disconnect_ = ffi.cast('_ServiceStatusCallback', function(name, topic) disconnect_cb(ffi.string(name), ffi.string(topic)) end)
  end

  local p = f.advertise(self.o, topic, queue_size or 1000, msg_spec:md5(), msg_spec.type, msg_spec.definition, msg_spec.has_header, latch or false, connect_, disconnect_, utils.cdata(callback_queue))
  return ros.Publisher(p, msg_spec, connect_, disconnect_, self.serialization_handlers)
end

--- Create a client for a service
-- @tparam string service_name Name of the service to contact
-- @tparam ?string|ros.SrvSpec service_spec Service specification
-- @tparam[opt=false] bool persistent Whether this connection should persist. Persistent services keep the connection to the remote host active so that subsequent calls will happen faster. In general persistent services are discouraged, as they are not as robust to node failure as non-persistent services.
-- @tparam[opt] tab header_values  Key/value pairs you'd like to send along in the connection handshake
function NodeHandle:serviceClient(service_name, service_spec, persistent, header_values)
  if type(service_spec) == 'string' then
    service_spec = ros.SrvSpec(service_spec)
  end
  if not torch.isTypeOf(service_spec, ros.SrvSpec) then
    error("NodeHandle:serviceClient(): invalid 'service_spec' argument.")
  end
  local client = f.serviceClient(self.o, service_name, service_spec:md5(), persistent or false, utils.cdata(header_values))
  return ros.ServiceClient(client, service_spec, nil, nil, self.serialization_handlers)
end

--- Advertise a service
-- This call connects to the master to publicize that the node will be offering an RPC service with the given name.
-- @tparam string service_name Name of the service
-- @tparam ros.SrvSpec service_spec Service specification
-- @tparam func service_handler_func Service handler function
-- @tparam[opt=ros.DEFAULT_CALLBACK_QUEUE] ros.CallbackQueue callback_queue ROS callback queue
-- @treturn ros.ServiceServer
function NodeHandle:advertiseService(service_name, service_spec, service_handler_func, callback_queue)
  callback_queue = callback_queue or ros.DEFAULT_CALLBACK_QUEUE
  if not torch.isTypeOf(callback_queue, ros.CallbackQueue) then
    error('Invalid type of explicitly specified callback queue.')
  end

  -- create message serialization/deserialization wrapper function
  local function handler(request_storage, response_storage, header_values)

    -- create torch.ByteStorage() obj from THByteStorage* and addref
    request_storage = torch.pushudata(request_storage, 'torch.ByteStorage')
    request_storage:retain()

    response_storage = ros.SerializedMessage.fromPtr(response_storage)

    -- create class around header values string map...
    local header = torch.factory('std.StringMap')()
    rawset(header, 'o', header_values)

    -- deserialize request
    local request_msg = ros.Message(service_spec.request_spec, true)
    request_msg:deserialize(ros.StorageReader(request_storage))
    local response_msg = ros.Message(service_spec.response_spec)

    -- call actual service handler function
    local ok, status = pcall(service_handler_func, request_msg, response_msg, header)
    if not ok then
      ros.ERROR(status)
      status = false
    end

    -- serialize response
    local sw = ros.StorageWriter(response_storage)
    local v = response_msg:serializeServiceResponse(sw, status)
    sw:shrinkToFit()

    return status
  end

  local cb = ffi.cast("ServiceRequestCallback", handler)
  local srv_ptr = f.advertiseService(
    self.o,
    service_name,
    service_spec:md5(),
    service_spec.type,
    service_spec.request_spec.type,
    service_spec.response_spec.type,
    cb,
    callback_queue:cdata()
  )
  return ros.ServiceServer(srv_ptr, cb, service_handler_func)
end

--- Check whether a parameter exists on the parameter server.
-- @tparam string key The key to check.
-- @treturn bool true if the parameter exists, false otherwise
function NodeHandle:hasParam(key)
  return f.hasParam(self.o, key)
end

--- Delete a parameter from the parameter server.
-- @tparam string key The key to delete.
-- @treturn bool true if the deletion succeeded, false otherwise.
function NodeHandle:deleteParam(key)
  return f.deleteParam(self.o, key)
end

--- Get the parameter value for key key.
-- @tparam string key The key to get the value for
-- @treturn string The key value
-- @treturn bool     true if the parameter value was retrieved, false otherwise
function NodeHandle:getParamString(key)
  local result = std.String()
  local ok = f.getParamString(self.o, key, result:cdata())
  return result:get(), ok
end

local double_ct = ffi.typeof('double[1]')
local float_ct = ffi.typeof('float[1]')
local int_ct = ffi.typeof('int[1]')
local bool_ct = ffi.typeof('bool[1]')

--- Get the parameter value for key key.
-- @tparam string key The key to get the value for
-- @treturn number The key value
-- @treturn bool true if the parameter value was retrieved, false otherwise
function NodeHandle:getParamDouble(key)
  local result = double_ct(0)
  local ok = f.getParamDouble(self.o, key, result)
  return result[0], ok
end

--- Get the parameter value for key key.
-- @tparam string key The key to get the value for
-- @treturn number The key value
-- @treturn bool true if the parameter value was retrieved, false otherwise
function NodeHandle:getParamFloat(key)
  local result = float_ct(0)
  local ok = f.getParamFloat(self.o, key, result)
  return result[0], ok
end

--- Get the parameter value for key key.
-- @tparam string key The key to get the value for
-- @treturn int The key value
-- @treturn bool true if the parameter value was retrieved, false otherwise
function NodeHandle:getParamInt(key)
  local result = int_ct(0)
  local ok = f.getParamInt(self.o, key, result)
  return result[0], ok
end

--- Get the parameter value for key key.
-- @tparam string key The key to get the value for
-- @treturn bool The key value
-- @treturn bool true if the parameter value was retrieved, false otherwise
function NodeHandle:getParamBool(key)
  local result = bool_ct(0)
  local ok = f.getParamBool(self.o, key, result)
  return result[0], ok
end

--- Set the parameter value for key key.
-- @tparam string key The key to set the value
-- @tparam string value The value of the key
-- @treturn bool true if the parameter value was set successfully, false otherwise
function NodeHandle:setParamString(key, value)
  f.setParamString(self.o, key, value)
end

--- Set the parameter value for key key.
-- @tparam string key The key to set the value
-- @tparam number value The value of the key
-- @treturn bool true if the parameter value was set successfully, false otherwise
function NodeHandle:setParamDouble(key, value)
  f.setParamDouble(self.o, key, value)
end

--- Set the parameter value for key key.
-- @tparam string key The key to set the value
-- @tparam number value The value of the key
-- @treturn bool true if the parameter value was set successfully, false otherwise
function NodeHandle:setParamFloat(key, value)
  f.setParamFloat(self.o, key, value)
end

--- Set the parameter value for key key.
-- @tparam string key The key to set the value
-- @tparam int value The value of the key
-- @treturn bool true if the parameter value was set successfully, false otherwise
function NodeHandle:setParamInt(key, value)
  f.setParamInt(self.o, key, value)
end

--- Set the parameter value for key key.
-- @tparam string key The key to set the value
-- @tparam bool value The value of the key
-- @treturn bool true if the parameter value was set successfully, false otherwise
function NodeHandle:setParamBool(key, value)
  f.setParamBool(self.o, key, value)
end

--- Get the parameter value for key key.
-- @tparam string key The key to get the value for
-- @treturn std.StringVector The key value
-- @treturn bool true if the parameters value was retrieved, false otherwise
function NodeHandle:getParamStringVector(key)
  local result = std.StringVector()
  local ok = f.getParamStringVector(self.o, key, result:cdata())
  return result, ok
end

--- Get the parameter value for key key.
-- @tparam string key The key to get the value for
-- @treturn torch.ByteTensor The key value
-- @treturn bool true if the parameters value was retrieved, false otherwise
function NodeHandle:getParamBoolVector(key)
  local result = torch.ByteTensor()
  local ok = f.getParamBoolVector(self.o, key, result:cdata())
  return result, ok
end

--- Get the parameter value for key key.
-- @tparam string key The key to get the value for
-- @treturn torch.IntTensor The key value
-- @treturn bool true if the parameters value was retrieved, false otherwise
function NodeHandle:getParamIntVector(key)
  local result = torch.IntTensor()
  local ok = f.getParamIntVector(self.o, key, result:cdata())
  return result, ok
end

--- Get the parameter value for key key.
-- @tparam string key The key to get the value for
-- @treturn torch.DoubleTensor The key value
-- @treturn bool true if the parameters value was retrieved, false otherwise
function NodeHandle:getParamDoubleVector(key)
  local result = torch.DoubleTensor()
  local ok = f.getParamDoubleVector(self.o, key, result:cdata())
  return result, ok
end

--- Get the parameter value for key key.
-- @tparam string key The key to get the value for
-- @treturn torch.FloatTensor The key value
-- @treturn bool true if the parameters value was retrieved, false otherwise
function NodeHandle:getParamFloatVector(key)
  local result = torch.FloatTensor()
  local ok = f.getParamDoubleVector(self.o, key, result:cdata())
  return result, ok
end

--- Set the parameter value for key key.
-- @tparam string key The key to set the value
-- @tparam std.StringVector value The value of the key
-- @treturn bool true if the parameter value was set successfully, false otherwise
function NodeHandle:setParamStringVector(key, value)
  f.setParamStringVector(self.o, key, value:cdata())
end

--- Set the parameter value for key key.
-- @tparam string key The key to set the value
-- @tparam torch.ByteTensor value The value of the key
-- @treturn bool true if the parameter value was set successfully, false otherwise
function NodeHandle:setParamBoolVector(key, value)
  f.setParamBoolVector(self.o, key, value:cdata())
end

--- Set the parameter value for key key.
-- @tparam string key The key to set the value
-- @tparam torch.IntTensor value The value of the key
-- @treturn bool true if the parameter value was set successfully, false otherwise
function NodeHandle:setParamIntVector(key, value)
  f.setParamIntVector(self.o, key, value:cdata())
end

--- Set the parameter value for key key.
-- @tparam string key The key to set the value
-- @tparam torch.DoubleTensor value The value of the key
-- @treturn bool true if the parameter value was set successfully, false otherwise
function NodeHandle:setParamDoubleVector(key, value)
  f.setParamDoubleVector(self.o, key, value:cdata())
end

--- Set the parameter value for key key.
-- @tparam string key The key to set the value
-- @tparam torch.FloatTensor value The value of the key
-- @treturn bool true if the parameter value was set successfully, false otherwise
function NodeHandle:setParamFloatVector(key, value)
  f.setParamFloatVector(self.o, key, value:cdata())
end
