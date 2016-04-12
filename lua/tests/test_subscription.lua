ros = require 'ros'

ros.init('rose')

spinner = ros.AsyncSpinner()
spinner:start()

nodehandle = ros.NodeHandle()

msgbuf = ros.MessageBuffer()

string_spec = ros.MsgSpec('std_msgs/String')

subscriber = nodehandle:subscribe("chatter", 'std_msgs/String', 100)

while ros.ok() do
  sys.sleep(0.1)
  while subscriber:hasMessage() do
    local msg = subscriber:read()
    print(msg)
  end
end
