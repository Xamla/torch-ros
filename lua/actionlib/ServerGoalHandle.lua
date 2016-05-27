local ros = require 'ros.env'
local std = ros.std

local actionlib = ros.actionlib

local ServerGoalHandle = torch.class('ros.actionlib.ServerGoalHandle', actionlib)


function ServerGoalHandle:__init(action_server)
  self.as = action_server
end


function ServerGoalHandle:setAccepted(text)
  ros.DEBUG_NAMED("actionlib", "Accepting goal, id: %s, stamp: %.2f", self:getGoalID().id, self:getGoalID().stamp:toSec())
  if self.goal ~= nil then
    boost::recursive_mutex::scoped_lock lock(as_->lock_);
    unsigned int status = (*status_it_).status_.status;

    -- if we were pending before, then we'll go active
    if (status == actionlib_msgs::GoalStatus::PENDING){
      (*status_it_).status_.status = actionlib_msgs::GoalStatus::ACTIVE;
      (*status_it_).status_.text = text;
      as_->publishStatus();
    }
    -- if we were recalling before, now we'll go to preempting
    else if (status == actionlib_msgs::GoalStatus::RECALLING){
      (*status_it_).status_.status = actionlib_msgs::GoalStatus::PREEMPTING;
      (*status_it_).status_.text = text;
      self.as:publishStatus()
    }
    else
      ros.ERROR_NAMED("actionlib", "To transition to an active state, the goal must be in a pending or recalling state, it is currently in state: %d",
          (*status_it_).status_.status);
    end
  }
  else
    ros.ERROR_NAMED("actionlib", "Attempt to set status on an uninitialized ServerGoalHandle");
  end
end


function ServerGoalHandle:setCanceled()
end


function ServerGoalHandle:setRejected()
end


function ServerGoalHandle:setAborted()
end


function ServerGoalHandle:setSucceeded()
end


function ServerGoalHandle:publishFeedback()
end


function ServerGoalHandle:isValid()
end


function ServerGoalHandle:getGoal()
end


function ServerGoalHandle:getGoalID()
end


function getGoalStatus()
end
