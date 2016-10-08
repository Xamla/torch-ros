local ros = require 'ros'

local function printf(...) return print(string.format(...)) end

ros.init('multi_array_demo')

local spinner = ros.AsyncSpinner()
spinner:start()

local nodehandle = ros.NodeHandle()


local publisher = nodehandle:advertise("float_tensor_source", 'std_msgs/Float64MultiArray', 10)
local subscriber = nodehandle:subscribe("float_tensor_source", 'std_msgs/Float64MultiArray', 10)


local specFloat64MultiArray = ros.MsgSpec('std_msgs/Float64MultiArray')

local function tensorToMsg(tensor)
  local msg = ros.Message(specFloat64MultiArray)
  msg.data = tensor:reshape(tensor:nElement())
  for i=1,tensor:dim() do
    local dim_desc = ros.Message('std_msgs/MultiArrayDimension')
    dim_desc.size = tensor:size(i)
    dim_desc.stride = tensor:stride(i)
    table.insert(msg.layout.dim, dim_desc)
  end
  return msg
end


local function msgToTensor(msg)
  -- TODO: add support non-continuous tensor (stride handling)
  local layout = msg.layout
  local dim_sizes = torch.LongStorage(#layout.dim)
  for i=1,#layout.dim do
    dim_sizes[i] = msg.layout.dim[i].size
  end
  return msg.data:reshape(dim_sizes)
end


local recv_seq = 1
subscriber:registerCallback(function(msg, header)
  --print('Header:')
  --print(header)
  --print('Message:')
  --print(msg)
  local t = msgToTensor(msg)
  --print("Tensor size:")
  --print(t:size())
  printf("[recv_seq: %d] Sum of all elements (received): %f", recv_seq, t:sum())
  recv_seq = recv_seq + 1
end)

local send_seq = 1
while ros.ok() do
  local source_data = torch.rand(4, 27, 13)
  local msg = tensorToMsg(source_data)
  --print("Sending message:")
  --print(msg)
  printf("[send_seq: %d] Sum of all elements (sent): %f", send_seq, source_data:sum())
  send_seq = send_seq + 1
  publisher:publish(msg)
  ros.spinOnce()
  sys.sleep(0.1)
end

ros.shutdown()
