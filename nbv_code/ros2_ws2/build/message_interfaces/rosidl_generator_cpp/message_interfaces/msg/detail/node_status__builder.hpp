// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from message_interfaces:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_INTERFACES__MSG__DETAIL__NODE_STATUS__BUILDER_HPP_
#define MESSAGE_INTERFACES__MSG__DETAIL__NODE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "message_interfaces/msg/detail/node_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace message_interfaces
{

namespace msg
{

namespace builder
{

class Init_NodeStatus_is_moving
{
public:
  Init_NodeStatus_is_moving()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::message_interfaces::msg::NodeStatus is_moving(::message_interfaces::msg::NodeStatus::_is_moving_type arg)
  {
    msg_.is_moving = std::move(arg);
    return std::move(msg_);
  }

private:
  ::message_interfaces::msg::NodeStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::message_interfaces::msg::NodeStatus>()
{
  return message_interfaces::msg::builder::Init_NodeStatus_is_moving();
}

}  // namespace message_interfaces

#endif  // MESSAGE_INTERFACES__MSG__DETAIL__NODE_STATUS__BUILDER_HPP_
