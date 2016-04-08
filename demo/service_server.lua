local ros = require 'ros'

ros.init('service_server')
nh = ros.NodeHandle()

service_queue = ros.CallbackQueue()

srv_spec = ros.SrvSpec('roscpp/GetLoggers')

print(srv_spec)

function myServiceHandler(request, header)
  print('[!] handler call')
  print('request:')
  print(request)
  print('header:')
  print(header)

  local respones = ros.Message(srv_spec.res_pspec)

  return true, response
end

server = nh:advertiseService('/demo_service', srv_spec, service_queue, myServiceHandler)
print('service server running, call "rosservice call /demo_service" to send a request to the service.')

while ros.ok() do
  sys.sleep(0.01)
  if not service_queue:isEmpty() then
    print('[!] incoming service call')
    service_queue:callAvailable()
  end
  ros.spinOnce()
end

server:shutdown()
ros.shutdown()
