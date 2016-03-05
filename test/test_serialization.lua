ros = require 'ros'

local header_spec = ros.MsgSpec('sensor_msgs/Image')

local msg = ros.Message(header_spec)
print(msg.spec)
print(msg)