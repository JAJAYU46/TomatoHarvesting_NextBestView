// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from message_interfaces:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_INTERFACES__MSG__DETAIL__NODE_STATUS__TRAITS_HPP_
#define MESSAGE_INTERFACES__MSG__DETAIL__NODE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "message_interfaces/msg/detail/node_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace message_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const NodeStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: ready_for_next_iteration
  {
    out << "ready_for_next_iteration: ";
    rosidl_generator_traits::value_to_yaml(msg.ready_for_next_iteration, out);
    out << ", ";
  }

  // member: is_moving
  {
    out << "is_moving: ";
    rosidl_generator_traits::value_to_yaml(msg.is_moving, out);
    out << ", ";
  }

  // member: iteration
  {
    out << "iteration: ";
    rosidl_generator_traits::value_to_yaml(msg.iteration, out);
    out << ", ";
  }

  // member: detection_done
  {
    out << "detection_done: ";
    rosidl_generator_traits::value_to_yaml(msg.detection_done, out);
    out << ", ";
  }

  // member: icp_done
  {
    out << "icp_done: ";
    rosidl_generator_traits::value_to_yaml(msg.icp_done, out);
    out << ", ";
  }

  // member: octomap_done
  {
    out << "octomap_done: ";
    rosidl_generator_traits::value_to_yaml(msg.octomap_done, out);
    out << ", ";
  }

  // member: nbv_done
  {
    out << "nbv_done: ";
    rosidl_generator_traits::value_to_yaml(msg.nbv_done, out);
    out << ", ";
  }

  // member: nbv_point_x
  {
    out << "nbv_point_x: ";
    rosidl_generator_traits::value_to_yaml(msg.nbv_point_x, out);
    out << ", ";
  }

  // member: nbv_point_y
  {
    out << "nbv_point_y: ";
    rosidl_generator_traits::value_to_yaml(msg.nbv_point_y, out);
    out << ", ";
  }

  // member: nbv_point_z
  {
    out << "nbv_point_z: ";
    rosidl_generator_traits::value_to_yaml(msg.nbv_point_z, out);
    out << ", ";
  }

  // member: is_final_result
  {
    out << "is_final_result: ";
    rosidl_generator_traits::value_to_yaml(msg.is_final_result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NodeStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ready_for_next_iteration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ready_for_next_iteration: ";
    rosidl_generator_traits::value_to_yaml(msg.ready_for_next_iteration, out);
    out << "\n";
  }

  // member: is_moving
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_moving: ";
    rosidl_generator_traits::value_to_yaml(msg.is_moving, out);
    out << "\n";
  }

  // member: iteration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "iteration: ";
    rosidl_generator_traits::value_to_yaml(msg.iteration, out);
    out << "\n";
  }

  // member: detection_done
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detection_done: ";
    rosidl_generator_traits::value_to_yaml(msg.detection_done, out);
    out << "\n";
  }

  // member: icp_done
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "icp_done: ";
    rosidl_generator_traits::value_to_yaml(msg.icp_done, out);
    out << "\n";
  }

  // member: octomap_done
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "octomap_done: ";
    rosidl_generator_traits::value_to_yaml(msg.octomap_done, out);
    out << "\n";
  }

  // member: nbv_done
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nbv_done: ";
    rosidl_generator_traits::value_to_yaml(msg.nbv_done, out);
    out << "\n";
  }

  // member: nbv_point_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nbv_point_x: ";
    rosidl_generator_traits::value_to_yaml(msg.nbv_point_x, out);
    out << "\n";
  }

  // member: nbv_point_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nbv_point_y: ";
    rosidl_generator_traits::value_to_yaml(msg.nbv_point_y, out);
    out << "\n";
  }

  // member: nbv_point_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nbv_point_z: ";
    rosidl_generator_traits::value_to_yaml(msg.nbv_point_z, out);
    out << "\n";
  }

  // member: is_final_result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_final_result: ";
    rosidl_generator_traits::value_to_yaml(msg.is_final_result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NodeStatus & msg, bool use_flow_style = false)
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
  const message_interfaces::msg::NodeStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  message_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use message_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const message_interfaces::msg::NodeStatus & msg)
{
  return message_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<message_interfaces::msg::NodeStatus>()
{
  return "message_interfaces::msg::NodeStatus";
}

template<>
inline const char * name<message_interfaces::msg::NodeStatus>()
{
  return "message_interfaces/msg/NodeStatus";
}

template<>
struct has_fixed_size<message_interfaces::msg::NodeStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<message_interfaces::msg::NodeStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<message_interfaces::msg::NodeStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGE_INTERFACES__MSG__DETAIL__NODE_STATUS__TRAITS_HPP_
