ros = require 'ros'

ros.init('rose')

spinner = ros.AsyncSpinner()
spinner:start()

nodehandle = ros.NodeHandle()

subscriber = nodehandle:subscribe("chatter", 'std_msgs/String', 100)

while ros.ok() do
  sys.sleep(0.1)
  while subscriber:hasMessage() do
    local msg = subscriber:read()
    print(msg)
  end
end

subscriber:shutdown()
ros.shutdown()
