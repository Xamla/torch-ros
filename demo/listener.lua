ros = require 'ros'

ros.init('listener')

spinner = ros.AsyncSpinner()
spinner:start()

nodehandle = ros.NodeHandle()

subscriber = nodehandle:subscribe("dummy_chat", 'std_msgs/String', 100, { 'udp', 'tcp' }, { tcp_nodelay = true })

while ros.ok() do
  ros.spinOnce()
  sys.sleep(0.1)
  while subscriber:hasMessage() do
    local msg = subscriber:read()
    print(msg)
  end
end

subscriber:shutdown()
ros.shutdown()
