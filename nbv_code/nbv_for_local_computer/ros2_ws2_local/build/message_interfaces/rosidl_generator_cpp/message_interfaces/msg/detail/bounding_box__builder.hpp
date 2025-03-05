// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from message_interfaces:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_INTERFACES__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
#define MESSAGE_INTERFACES__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "message_interfaces/msg/detail/bounding_box__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace message_interfaces
{

namespace msg
{

namespace builder
{

class Init_BoundingBox_rd_y
{
public:
  explicit Init_BoundingBox_rd_y(::message_interfaces::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  ::message_interfaces::msg::BoundingBox rd_y(::message_interfaces::msg::BoundingBox::_rd_y_type arg)
  {
    msg_.rd_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::message_interfaces::msg::BoundingBox msg_;
};

class Init_BoundingBox_rd_x
{
public:
  explicit Init_BoundingBox_rd_x(::message_interfaces::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_rd_y rd_x(::message_interfaces::msg::BoundingBox::_rd_x_type arg)
  {
    msg_.rd_x = std::move(arg);
    return Init_BoundingBox_rd_y(msg_);
  }

private:
  ::message_interfaces::msg::BoundingBox msg_;
};

class Init_BoundingBox_lu_y
{
public:
  explicit Init_BoundingBox_lu_y(::message_interfaces::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_rd_x lu_y(::message_interfaces::msg::BoundingBox::_lu_y_type arg)
  {
    msg_.lu_y = std::move(arg);
    return Init_BoundingBox_rd_x(msg_);
  }

private:
  ::message_interfaces::msg::BoundingBox msg_;
};

class Init_BoundingBox_lu_x
{
public:
  Init_BoundingBox_lu_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBox_lu_y lu_x(::message_interfaces::msg::BoundingBox::_lu_x_type arg)
  {
    msg_.lu_x = std::move(arg);
    return Init_BoundingBox_lu_y(msg_);
  }

private:
  ::message_interfaces::msg::BoundingBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::message_interfaces::msg::BoundingBox>()
{
  return message_interfaces::msg::builder::Init_BoundingBox_lu_x();
}

}  // namespace message_interfaces

#endif  // MESSAGE_INTERFACES__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
