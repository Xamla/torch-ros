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

function ActionClient:__init(node_handle, name, callback_queue)
  self.nh = node_handle
  self.name = name
  initClient(self, callback_queue)
end

function ActionClient:sendGoal(goal, transition_cb, feedback_cb)
  ros.DEBUG_NAMED("actionlib", "about to start initGoal()")
  local gh = self.manager:initGoal(goal, transition_cb, feedback_cb)
  ros.DEBUG_NAMED("actionlib", "Done with initGoal()")
  return gh
end

function ActionClient:cancelAllGoals()
--[[
    actionlib_msgs::GoalID cancel_msg;
    // CancelAll policy encoded by stamp=0, id=0
    cancel_msg.stamp = ros::Time(0,0);
    cancel_msg.id = "";
    cancel_pub_.publish(cancel_msg);
    ]]
end

--[[
 void cancelGoalsAtAndBeforeTime(const ros::Time& time)
  {
    actionlib_msgs::GoalID cancel_msg;
    cancel_msg.stamp = time;
    cancel_msg.id = "";
    cancel_pub_.publish(cancel_msg);
  }

bool waitForActionServerToStart(const ros::Duration& timeout = ros::Duration(0,0) )
]]
