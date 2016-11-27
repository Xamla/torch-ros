ros = require 'ros'
require 'ros.actionlib.ActionSpec'
require 'ros.actionlib.ActionClient'


ros.init('call_movegroup_demo')
ros.console.initialize()
ros.console.get_logger('ActionClient')
ros.console.set_logger_level('ActionClient', ros.console.Level.Debug)
ros.DEBUG_NAMED('ActionClient', 'bulb')

local move_group_action_spec = ros.actionlib.ActionSpec('moveit_msgs/MoveGroup')

local client = ros.actionlib.ActionClient(move_group_action_spec, 'move_group')

print('waiting for action server to start ...')
client:waitForActionServerToStart()
print('ready.')

ros.shutdown()
