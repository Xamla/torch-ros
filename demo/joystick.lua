--[[
Subscribe to joystick input and dump messages.

1. install joy package:
$ sudo apt-get install ros-indigo-joy

2. configure input device to use:
$ rosparam set joy_node/dev "/dev/input/jsX"

3. run joy node:
$ rosrun joy joy_node

further help to get joy package running:
http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick
]]

ros = require 'ros'

ros.init('joystick_demo')
nh = ros.NodeHandle()

joy = nh:subscribe('joy', 'sensor_msgs/Joy', 100)
print('Subscribed to \'joy\' node. Please start using your joystick.')
--print(joy.msg_spec)

d = ros.Duration(0.0025)
while ros.ok() do
  ros.spinOnce()
  d:sleep()
  while joy:hasMessage() do
    local msg = joy:read()
    print(msg)
  end
end

joy:shutdown()
ros.shutdown()
