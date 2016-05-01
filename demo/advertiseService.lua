--[[
Advertise a service from torch-ros.
The existing roscpp/GetLoggers srv file ist used.
]]

local ros = require 'ros'

ros.init('advertiseService_demo')
nh = ros.NodeHandle()

service_queue = ros.CallbackQueue()

srv_spec = ros.SrvSpec('roscpp/GetLoggers')
print(srv_spec)

function myServiceHandler(request, response, header)
  print('[!] handler call')
  print('request:')
  print(request)
  print('header:')
  print(header)

  for k,v in pairs(ros.console.level) do
    local l = ros.Message('roscpp/Logger')
    l.name = 'dummyname' .. v
    l.level = k
    table.insert(response.loggers, l)
  end

  print('response:')
  print(response)

  return true
end

server = nh:advertiseService('/demo_service', srv_spec, myServiceHandler, service_queue)
print('name: ' .. server:getService())
print('service server running, call "rosservice call /demo_service" to send a request to the service.')

local s = ros.Duration(0.001)
while ros.ok() do
  s:sleep()
  if not service_queue:isEmpty() then
    print('[!] incoming service call')
    service_queue:callAvailable()
  end
  ros.spinOnce()
end

server:shutdown()
ros.shutdown()
