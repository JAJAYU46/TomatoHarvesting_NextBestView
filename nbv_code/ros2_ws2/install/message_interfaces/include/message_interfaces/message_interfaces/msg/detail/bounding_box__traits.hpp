// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from message_interfaces:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_INTERFACES__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
#define MESSAGE_INTERFACES__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "message_interfaces/msg/detail/bounding_box__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace message_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const BoundingBox & msg,
  std::ostream & out)
{
  out << "{";
  // member: lu_x
  {
    out << "lu_x: ";
    rosidl_generator_traits::value_to_yaml(msg.lu_x, out);
    out << ", ";
  }

  // member: lu_y
  {
    out << "lu_y: ";
    rosidl_generator_traits::value_to_yaml(msg.lu_y, out);
    out << ", ";
  }

  // member: rd_x
  {
    out << "rd_x: ";
    rosidl_generator_traits::value_to_yaml(msg.rd_x, out);
    out << ", ";
  }

  // member: rd_y
  {
    out << "rd_y: ";
    rosidl_generator_traits::value_to_yaml(msg.rd_y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BoundingBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: lu_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lu_x: ";
    rosidl_generator_traits::value_to_yaml(msg.lu_x, out);
    out << "\n";
  }

  // member: lu_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lu_y: ";
    rosidl_generator_traits::value_to_yaml(msg.lu_y, out);
    out << "\n";
  }

  // member: rd_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rd_x: ";
    rosidl_generator_traits::value_to_yaml(msg.rd_x, out);
    out << "\n";
  }

  // member: rd_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rd_y: ";
    rosidl_generator_traits::value_to_yaml(msg.rd_y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BoundingBox & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace message_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use message_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const message_interfaces::msg::BoundingBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  message_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use message_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const message_interfaces::msg::BoundingBox & msg)
{
  return message_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<message_interfaces::msg::BoundingBox>()
{
  return "message_interfaces::msg::BoundingBox";
}

template<>
inline const char * name<message_interfaces::msg::BoundingBox>()
{
  return "message_interfaces/msg/BoundingBox";
}

template<>
struct has_fixed_size<message_interfaces::msg::BoundingBox>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<message_interfaces::msg::BoundingBox>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<message_interfaces::msg::BoundingBox>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGE_INTERFACES__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
