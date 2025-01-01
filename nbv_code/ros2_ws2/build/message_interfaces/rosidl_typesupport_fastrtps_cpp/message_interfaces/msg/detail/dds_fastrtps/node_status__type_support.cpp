// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from message_interfaces:msg/NodeStatus.idl
// generated code does not contain a copyright notice
#include "message_interfaces/msg/detail/node_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "message_interfaces/msg/detail/node_status__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace message_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_message_interfaces
cdr_serialize(
  const message_interfaces::msg::NodeStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: ready_for_next_iteration
  cdr << (ros_message.ready_for_next_iteration ? true : false);
  // Member: is_moving
  cdr << (ros_message.is_moving ? true : false);
  // Member: iteration
  cdr << ros_message.iteration;
  // Member: detection_done
  cdr << (ros_message.detection_done ? true : false);
  // Member: icp_done
  cdr << (ros_message.icp_done ? true : false);
  // Member: octomap_done
  cdr << (ros_message.octomap_done ? true : false);
  // Member: nbv_done
  cdr << (ros_message.nbv_done ? true : false);
  // Member: nbv_point_x
  cdr << ros_message.nbv_point_x;
  // Member: nbv_point_y
  cdr << ros_message.nbv_point_y;
  // Member: nbv_point_z
  cdr << ros_message.nbv_point_z;
  // Member: is_final_result
  cdr << (ros_message.is_final_result ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_message_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  message_interfaces::msg::NodeStatus & ros_message)
{
  // Member: ready_for_next_iteration
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.ready_for_next_iteration = tmp ? true : false;
  }

  // Member: is_moving
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_moving = tmp ? true : false;
  }

  // Member: iteration
  cdr >> ros_message.iteration;

  // Member: detection_done
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.detection_done = tmp ? true : false;
  }

  // Member: icp_done
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.icp_done = tmp ? true : false;
  }

  // Member: octomap_done
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.octomap_done = tmp ? true : false;
  }

  // Member: nbv_done
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.nbv_done = tmp ? true : false;
  }

  // Member: nbv_point_x
  cdr >> ros_message.nbv_point_x;

  // Member: nbv_point_y
  cdr >> ros_message.nbv_point_y;

  // Member: nbv_point_z
  cdr >> ros_message.nbv_point_z;

  // Member: is_final_result
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_final_result = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_message_interfaces
get_serialized_size(
  const message_interfaces::msg::NodeStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: ready_for_next_iteration
  {
    size_t item_size = sizeof(ros_message.ready_for_next_iteration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: is_moving
  {
    size_t item_size = sizeof(ros_message.is_moving);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: iteration
  {
    size_t item_size = sizeof(ros_message.iteration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: detection_done
  {
    size_t item_size = sizeof(ros_message.detection_done);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: icp_done
  {
    size_t item_size = sizeof(ros_message.icp_done);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: octomap_done
  {
    size_t item_size = sizeof(ros_message.octomap_done);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: nbv_done
  {
    size_t item_size = sizeof(ros_message.nbv_done);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: nbv_point_x
  {
    size_t item_size = sizeof(ros_message.nbv_point_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: nbv_point_y
  {
    size_t item_size = sizeof(ros_message.nbv_point_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: nbv_point_z
  {
    size_t item_size = sizeof(ros_message.nbv_point_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: is_final_result
  {
    size_t item_size = sizeof(ros_message.is_final_result);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_message_interfaces
max_serialized_size_NodeStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: ready_for_next_iteration
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: is_moving
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: iteration
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: detection_done
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: icp_done
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: octomap_done
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: nbv_done
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: nbv_point_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: nbv_point_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: nbv_point_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: is_final_result
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = message_interfaces::msg::NodeStatus;
    is_plain =
      (
      offsetof(DataType, is_final_result) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _NodeStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const message_interfaces::msg::NodeStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _NodeStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<message_interfaces::msg::NodeStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _NodeStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const message_interfaces::msg::NodeStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _NodeStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_NodeStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _NodeStatus__callbacks = {
  "message_interfaces::msg",
  "NodeStatus",
  _NodeStatus__cdr_serialize,
  _NodeStatus__cdr_deserialize,
  _NodeStatus__get_serialized_size,
  _NodeStatus__max_serialized_size
};

static rosidl_message_type_support_t _NodeStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_NodeStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace message_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_message_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<message_interfaces::msg::NodeStatus>()
{
  return &message_interfaces::msg::typesupport_fastrtps_cpp::_NodeStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, message_interfaces, msg, NodeStatus)() {
  return &message_interfaces::msg::typesupport_fastrtps_cpp::_NodeStatus__handle;
}

#ifdef __cplusplus
}
#endif
