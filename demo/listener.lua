ros = require 'ros'

ros.init('listener')

spinner = ros.AsyncSpinner()
spinner:start()

nodehandle = ros.NodeHandle()

msgbuf = ros.MessageBuffer()

string_spec = ros.MsgSpec('std_msgs/String')

nodehandle:subscribe("dummy_chat", string_spec, 100, msgbuf)

while ros.ok() do
  sys.sleep(0.1)
  while msgbuf:count() > 0 do
    local msg_bytes = msgbuf:read()
    local msg = ros.Message(string_spec, true)
    msg:deserialize(msg_bytes)
    print(msg)
  end
end
