# torch-ros

Torch7/lua wrapper of roscpp via ffi. It offers dynamic serialization of ROS message without requiring pre-generated message classes.

## Currently supported feature set:

- dynamic interpretation of ROS messages specifications (MsgSpec)
- serialization/deserialization of ROS messages
- representation of certain array types (byte, char, short, int, long, float, double) in messages as torch.Tensors
- publishing and subscribing to ROS topics
- basic parameter server support (NodeHandle:getParam*())
- basic ROS time support
- basic ROS-console/logging support (e.g. ros.WARN(), ros.INFO() etc.)
- service calls (advertiseService & serviceClient)
- ActionLib (ActionClient/ActionServer & SimpleActionClient/SimpleActionServer)
- TF library (Transform, StampedTransform, Quaternion, TransformListener, TransformBroadcaster)
- wrappers for basic std::string, std::vector&lt;std::string&gt;

## Todo:

- C++ exception handling/translation to lua

## Limitations

Currently ony little-endian systems are supported.

## Based on roslua 

This project uses fragments of [roslua](https://github.com/timn/roslua) which was developed by Tim Niemueller @timn.
