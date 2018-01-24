ros = require 'ros'

ros.init('logging_demo')

-- very simple logging
ros.FATAL('Oh, noooo...')
ros.ERROR('Something bad happened.')
ros.WARN('This is a warning.')
ros.INFO('I would like to inform you about the current state.')
ros.DEBUG('A very verbose debug message with useless numeric output: %d', 123)

ros.DEBUG_COND(true, 'Messages can also have conditions')
ros.DEBUG_COND(false, 'This message will not be printed...')

for i = 1, 10 do
  ros.ROS_ERROR_ONCE_NAMED('throttle', "This messege will be printed once")
  ros.ROS_INFO_THROTTLE_NAMED(5,'throttle', "This messege will be printed every 5 seconds")
  ros.ROS_WARN_THROTTLE_NAMED(2,'throttle1', "This messege will be printed every 2 seconds")
  sys.sleep(2)
end

-- get name and log level of registered loggers
local names, levels = ros.console.get_loggers()
ros.INFO('Names and levels of loggers: %s %s', names, levels)

ros.shutdown()
