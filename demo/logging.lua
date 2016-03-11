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

-- get name and log level of registered loggers
local names, levels = ros.console.get_loggers()
ros.INFO('Names and levels of loggers: %s %s', names, levels)

ros.shutdown()
