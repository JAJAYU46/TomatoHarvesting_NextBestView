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

class Init_NodeStatus_is_final_result
{
public:
  explicit Init_NodeStatus_is_final_result(::message_interfaces::msg::NodeStatus & msg)
  : msg_(msg)
  {}
  ::message_interfaces::msg::NodeStatus is_final_result(::message_interfaces::msg::NodeStatus::_is_final_result_type arg)
  {
    msg_.is_final_result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::message_interfaces::msg::NodeStatus msg_;
};

class Init_NodeStatus_nbv_point_z
{
public:
  explicit Init_NodeStatus_nbv_point_z(::message_interfaces::msg::NodeStatus & msg)
  : msg_(msg)
  {}
  Init_NodeStatus_is_final_result nbv_point_z(::message_interfaces::msg::NodeStatus::_nbv_point_z_type arg)
  {
    msg_.nbv_point_z = std::move(arg);
    return Init_NodeStatus_is_final_result(msg_);
  }

private:
  ::message_interfaces::msg::NodeStatus msg_;
};

class Init_NodeStatus_nbv_point_y
{
public:
  explicit Init_NodeStatus_nbv_point_y(::message_interfaces::msg::NodeStatus & msg)
  : msg_(msg)
  {}
  Init_NodeStatus_nbv_point_z nbv_point_y(::message_interfaces::msg::NodeStatus::_nbv_point_y_type arg)
  {
    msg_.nbv_point_y = std::move(arg);
    return Init_NodeStatus_nbv_point_z(msg_);
  }

private:
  ::message_interfaces::msg::NodeStatus msg_;
};

class Init_NodeStatus_nbv_point_x
{
public:
  explicit Init_NodeStatus_nbv_point_x(::message_interfaces::msg::NodeStatus & msg)
  : msg_(msg)
  {}
  Init_NodeStatus_nbv_point_y nbv_point_x(::message_interfaces::msg::NodeStatus::_nbv_point_x_type arg)
  {
    msg_.nbv_point_x = std::move(arg);
    return Init_NodeStatus_nbv_point_y(msg_);
  }

private:
  ::message_interfaces::msg::NodeStatus msg_;
};

class Init_NodeStatus_nbv_done
{
public:
  explicit Init_NodeStatus_nbv_done(::message_interfaces::msg::NodeStatus & msg)
  : msg_(msg)
  {}
  Init_NodeStatus_nbv_point_x nbv_done(::message_interfaces::msg::NodeStatus::_nbv_done_type arg)
  {
    msg_.nbv_done = std::move(arg);
    return Init_NodeStatus_nbv_point_x(msg_);
  }

private:
  ::message_interfaces::msg::NodeStatus msg_;
};

class Init_NodeStatus_octomap_done
{
public:
  explicit Init_NodeStatus_octomap_done(::message_interfaces::msg::NodeStatus & msg)
  : msg_(msg)
  {}
  Init_NodeStatus_nbv_done octomap_done(::message_interfaces::msg::NodeStatus::_octomap_done_type arg)
  {
    msg_.octomap_done = std::move(arg);
    return Init_NodeStatus_nbv_done(msg_);
  }

private:
  ::message_interfaces::msg::NodeStatus msg_;
};

class Init_NodeStatus_icp_done
{
public:
  explicit Init_NodeStatus_icp_done(::message_interfaces::msg::NodeStatus & msg)
  : msg_(msg)
  {}
  Init_NodeStatus_octomap_done icp_done(::message_interfaces::msg::NodeStatus::_icp_done_type arg)
  {
    msg_.icp_done = std::move(arg);
    return Init_NodeStatus_octomap_done(msg_);
  }

private:
  ::message_interfaces::msg::NodeStatus msg_;
};

class Init_NodeStatus_detection_done
{
public:
  explicit Init_NodeStatus_detection_done(::message_interfaces::msg::NodeStatus & msg)
  : msg_(msg)
  {}
  Init_NodeStatus_icp_done detection_done(::message_interfaces::msg::NodeStatus::_detection_done_type arg)
  {
    msg_.detection_done = std::move(arg);
    return Init_NodeStatus_icp_done(msg_);
  }

private:
  ::message_interfaces::msg::NodeStatus msg_;
};

class Init_NodeStatus_iteration
{
public:
  explicit Init_NodeStatus_iteration(::message_interfaces::msg::NodeStatus & msg)
  : msg_(msg)
  {}
  Init_NodeStatus_detection_done iteration(::message_interfaces::msg::NodeStatus::_iteration_type arg)
  {
    msg_.iteration = std::move(arg);
    return Init_NodeStatus_detection_done(msg_);
  }

private:
  ::message_interfaces::msg::NodeStatus msg_;
};

class Init_NodeStatus_is_moving
{
public:
  explicit Init_NodeStatus_is_moving(::message_interfaces::msg::NodeStatus & msg)
  : msg_(msg)
  {}
  Init_NodeStatus_iteration is_moving(::message_interfaces::msg::NodeStatus::_is_moving_type arg)
  {
    msg_.is_moving = std::move(arg);
    return Init_NodeStatus_iteration(msg_);
  }

private:
  ::message_interfaces::msg::NodeStatus msg_;
};

class Init_NodeStatus_ready_for_next_iteration
{
public:
  Init_NodeStatus_ready_for_next_iteration()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NodeStatus_is_moving ready_for_next_iteration(::message_interfaces::msg::NodeStatus::_ready_for_next_iteration_type arg)
  {
    msg_.ready_for_next_iteration = std::move(arg);
    return Init_NodeStatus_is_moving(msg_);
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
  return message_interfaces::msg::builder::Init_NodeStatus_ready_for_next_iteration();
}

}  // namespace message_interfaces

#endif  // MESSAGE_INTERFACES__MSG__DETAIL__NODE_STATUS__BUILDER_HPP_
