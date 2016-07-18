--[[
This demo shows how to subscribe to a message topic and receive
incoming messages (in this case simple std_msg/String messages).
You can run `publish.lua` as a message generating counter part
to send string messages that are displayed by this demo.
]]

ros = require 'ros'


ros.init('subscribe_demo')

spinner = ros.AsyncSpinner()
spinner:start()

nodehandle = ros.NodeHandle()

-- subscribe to dummy_chat topic with 100 messages back-log
-- transport_options (arguments 4 & 5) are optional - used here only for demonstration purposes
subscriber = nodehandle:subscribe("dummy_chat", 'std_msgs/String', 100, { 'udp', 'tcp' }, { tcp_nodelay = true })

-- register a callback function that will be triggered from ros.spinOnce() when a message is available.
subscriber:registerCallback(function(msg, header)
  print('Header:')
  print(header)
  print('Message:')
  print(msg)
end)

while ros.ok() do
  ros.spinOnce()
  sys.sleep(0.1)
end

subscriber:shutdown()
ros.shutdown()
