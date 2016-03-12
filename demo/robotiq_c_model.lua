ros = require 'ros'

ros.init('robotiq_c_model_demo')
nodehandle = ros.NodeHandle()

grippers = {}

RobotiqCModel = torch.class('grippers.RobotiqCModel', grippers)

local function clamp(x, lo, hi)
  if x < lo then
    return lo
  elseif x > hi then
    return hi
  else
    return x
  end
end

local CModel_robot_input_spec = ros.MsgSpec('robotiq_c_model_control/CModel_robot_input')
local CModel_robot_output_spec = ros.MsgSpec('robotiq_c_model_control/CModel_robot_output')

function RobotiqCModel:__init(nodehandle)
  self.nodehandle = nodehandle
  self.msgbuf = ros.MessageBuffer()
  self.nodehandle:subscribe("/CModelRobotInput", CModel_robot_input_spec, 100, self.msgbuf )
  self.last_state = ros.Message(CModel_robot_input_spec)
  self.publisher = nodehandle:advertise("/CModelRobotOutput", CModel_robot_output_spec, 100)
  self:reset()
end

function RobotiqCModel:reset()
  self.cmd = ros.Message(CModel_robot_output_spec)
  self.cmd.rACT = 0
  self.publisher:publish(self.cmd)
end

function RobotiqCModel:publish()
  self.publisher:publish(self.cmd)
end

function RobotiqCModel:activate()
  self.cmd = ros.Message(CModel_robot_output_spec)
  self.cmd.rACT = 1
  self.cmd.rGTO = 1
  self.cmd.rSP  = 255
  self.cmd.rFR  = 10
  self:publish()
end

function RobotiqCModel:setSpeed(speed)
  self.cmd.rSP = clamp(speed, 0, 255)
  self:publish()
end

function RobotiqCModel:setForce(force)
  self.cmd.rFR = clamp(force, 0, 255)
  self:publish()
end

function RobotiqCModel:setPosition(pos)
  self.cmd.rPR = clamp(pos, 0, 255)
  self:publish()
end

function RobotiqCModel:open()
  self:setPosition(0)
end

function RobotiqCModel:close()
  self:setPosition(255)
end

function RobotiqCModel:spin()
  -- process available messages
  while self.msgbuf:count() > 0 do
    local msg_bytes = self.msgbuf:read()
    self.last_state:deserialize(msg_bytes)
  end
end

function RobotiqCModel:__tostring()
  return tostring(self.last_state)
end

spinner = ros.AsyncSpinner()
spinner:start()

gripper = grippers.RobotiqCModel(nodehandle)

ros.spinOnce()
gripper:reset()
sys.sleep(1)
gripper:activate()
gripper:setSpeed(5)
gripper:setForce(0)
ros.spinOnce()
gripper:open()
ros.spinOnce()
sys.sleep(2)
gripper:close()
ros.spinOnce()
sys.sleep(2)

