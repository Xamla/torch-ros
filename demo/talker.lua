ros = require 'ros'

ros.init('talker')

spinner = ros.AsyncSpinner()
spinner:start()

nodehandle = ros.NodeHandle()

string_spec = ros.MsgSpec('std_msgs/String')

publisher = nodehandle:advertise("dummy_chat", string_spec, 100)
ros.spinOnce()

m = ros.Message(string_spec)

function run(n)
  for i=1,n do
    if not ros.ok() then
      return
    end
    if publisher:getNumSubscribers() == 0 then
      print('waiting for subscriber')
    else
      m.data = "Hello this is a string message " .. i
      publisher:publish(m)
      print(i)
    end
    sys.sleep(0.1)
    ros.spinOnce()
  end
end

run(100)

ros.shutdown()
