--[[
Publish an uncompressed image via torch-ros.
This sample uses the torch image package as source for the Lena test image.
To show the result simply start rviz and click 'Add' and select 'By topic'.
In the treeview under topic 'lena_image' select 'Image'. A blinking Lena image
will be displayed. The image is inverted at each time step to create a
clearly visible change between consecutive frames.
]]

ros = require 'ros'
image = require 'image'

ros.init('lena_image_source')
nh = ros.NodeHandle()

spinner = ros.AsyncSpinner()
spinner:start()

-- convert into byte rgb and publish
local lena = image.lena()
lena = lena:mul(255):permute(2,3,1):byte()

publisher = nh:advertise("lena_image", 'sensor_msgs/Image', 10)

local msg = ros.Message('sensor_msgs/Image')

--[[
Message Definition of 'the sensor_msgs/Image'
(for details see: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)

std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
]]

msg.height = lena:size()[1]
msg.width = lena:size()[2]
msg.encoding = "rgb8"
msg.is_bigendian = false
msg.step = lena:stride()[1]
msg.data = lena:reshape(msg.height * msg.width * 3)

print('press ctrl+c to exit')
while ros.ok() do
  if publisher:getNumSubscribers() > 0 then
    lena = -lena -- invert image to get some blinking effect
    msg.data = lena:reshape(msg.height * msg.width * 3)
    publisher:publish(msg)
  end
  sys.sleep(0.1)
  ros.spinOnce()
end

ros.shutdown()
