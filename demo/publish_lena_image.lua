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

for i=1,100 do
  while ros.ok() do
    if publisher:getNumSubscribers() > 0 then
      publisher:publish(msg)
    end
    sys.sleep(0.1)
    ros.spinOnce()
  end
end

ros.shutdown()
