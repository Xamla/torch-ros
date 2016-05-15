local ros = require 'ros.env'
local std = ros.std

local actionlib = ros.actionlib

local CommState = {
  WAITING_FOR_GOAL_ACK    = 1,
  PENDING                 = 2,
  ACTIVE                  = 3,
  WAITING_FOR_RESULT      = 4,
  WAITING_FOR_CANCEL_ACK  = 5,
  RECALLING               = 6,
  PREEMPTING              = 7,
  DONE                    = 8,
  'WAITING_FOR_GOAL_ACK', 'PENDING', 'ACTIVE', 'WAITING_FOR_RESULT', 'WAITING_FOR_CANCEL_ACK', 'RECALLING', 'PREEMPTING', 'DONE'
}

local SimpleGoalState = {
  PENDING                 = 1,
  ACTIVE                  = 2,
  DONE                    = 3,
  'PENDING', 'ACTIVE', 'DONE'
}

local SimpleClientGoalState = {
  PENDING                 = 1,
  ACTIVE                  = 2,
  RECALLED                = 3,
  REJECTED                = 4,
  PREEMPTED               = 5,
  ABORTED                 = 6,
  SUCCEEDED               = 7,
  LOST                    = 8,
  'PENDING', 'ACTIVE', 'RECALLED', 'REJECTED', 'PREEMPTED', 'ABORTED', 'SUCCEEDED', 'LOST'
}

local TerminalState = {
  RECALLED                = 1,
  REJECTED                = 2,
  PREEMPTED               = 3,
  ABORTED                 = 4,
  SUCCEEDED               = 5,
  LOST                    = 6,
  'RECALLED', 'REJECTED', 'PREEMPTED', 'ABORTED', 'SUCCEEDED', 'LOST'
}

local ActionClient = torch.class('ros.actionlib.ActionServer', actionlib)

local function initClient(self, queue)
--[[  self.status_sub   = queue_subscribe("status",   1, &ActionClientT::statusCb,   this, queue);
  self.feedback_sub = queue_subscribe("feedback", 1, &ActionClientT::feedbackCb, this, queue);
  self.result_sub   = queue_subscribe("result",   1, &ActionClientT::resultCb,   this, queue);

  connection_monitor_.reset(new ConnectionMonitor(feedback_sub_, status_sub_));

    -- Start publishers and subscribers
    goal_pub_ = queue_advertise<ActionGoal>("goal", 10,
                                            boost::bind(&ConnectionMonitor::goalConnectCallback,    connection_monitor_, _1),
                                            boost::bind(&ConnectionMonitor::goalDisconnectCallback, connection_monitor_, _1),
                                            queue);
    cancel_pub_ = queue_advertise<actionlib_msgs::GoalID>("cancel", 10,
                                            boost::bind(&ConnectionMonitor::cancelConnectCallback,    connection_monitor_, _1),
                                            boost::bind(&ConnectionMonitor::cancelDisconnectCallback, connection_monitor_, _1),
                                            queue);

  manager_.registerSendGoalFunc(boost::bind(&ActionClientT::sendGoalFunc, this, _1));
  manager_.registerCancelFunc(boost::bind(&ActionClientT::sendCancelFunc, this, _1));
  ]]
end

local function goalConnectCallback(self, name, topic)
end

local function goalDisconnectCallback(self, name, topic)
end

local function cancelConnectCallback(self, name, topic)
end

local function cancelDisconnectCallback(self, name, topic)
end

function ActionClient:__init(action_spec, node_handle, name, callback_queue)
  self.action_spec = action_spec
  self.nh = node_handle
  self.name = name
  self.connection_monitor = {}

  self.status_sub   = self.nh:subscribe("status", "actionlib_msgs/GoalStatusArray", 50)
  self.feedback_sub = self.nh:subscribe("feedback", action_spec.action_feedback_spec, 50)
  self.result_sub   = self.nh:subscribe("result", action_spec.action_result_spec, 50)

  self.goal_pub = self.nh.advertise("goal", action_spec.action_goal_spec, 10, false,
    function(name, topic) goalConnectCallback(self, name, topic) end,
    function(name, topic) goalDisconnectCallback(self, name, topic) end,
    callback_queue
  )

  self.cancel_pub = self.nh.advertise("cancel", "actionlib_msgs/GoalID", 10, false,
    function(name, topic) cancelConnectCallback(self, name, topic) end,
    function(name, topic) cancelDisconnectCallback(self, name, topic) end,
    callback_queue
  )
end

function ActionClient:sendGoal(goal, transition_cb, feedback_cb)
  ros.DEBUG_NAMED("actionlib", "about to start initGoal()")
  local gh = self.manager:initGoal(goal, transition_cb, feedback_cb)
  ros.DEBUG_NAMED("actionlib", "Done with initGoal()")
  return gh
end

function ActionClient:cancelGoalsAtAndBeforeTime(time)
  local cancel_msg = ros.Message('actionlib_msgs/GoalID')
  cancel_msg.stamp = time
  self.cancel_pub:publish(cancel_msg)
end

function ActionClient:cancelAllGoals()
  self:cancelGoalsAtAndBeforeTime(ros.Time())     -- CancelAll is encoded by stamp=0
end

function ActionClient:waitForActionServerToStart(timeout)
  timeout = timeout or ros.Duration(0, 0)

end

function ActionClient:isServerConnected()
end
