// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from message_interfaces:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_INTERFACES__MSG__DETAIL__NODE_STATUS__STRUCT_HPP_
#define MESSAGE_INTERFACES__MSG__DETAIL__NODE_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__message_interfaces__msg__NodeStatus __attribute__((deprecated))
#else
# define DEPRECATED__message_interfaces__msg__NodeStatus __declspec(deprecated)
#endif

namespace message_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NodeStatus_
{
  using Type = NodeStatus_<ContainerAllocator>;

  explicit NodeStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ready_for_next_iteration = false;
      this->is_moving = false;
      this->target_box_id = 0l;
      this->iteration = 0l;
      this->detection_done = false;
      this->icp_done = false;
      this->octomap_done = false;
      this->nbv_done = false;
      this->nbv_point_x = 0.0;
      this->nbv_point_y = 0.0;
      this->nbv_point_z = 0.0;
      this->is_final_result = false;
    }
  }

  explicit NodeStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ready_for_next_iteration = false;
      this->is_moving = false;
      this->target_box_id = 0l;
      this->iteration = 0l;
      this->detection_done = false;
      this->icp_done = false;
      this->octomap_done = false;
      this->nbv_done = false;
      this->nbv_point_x = 0.0;
      this->nbv_point_y = 0.0;
      this->nbv_point_z = 0.0;
      this->is_final_result = false;
    }
  }

  // field types and members
  using _ready_for_next_iteration_type =
    bool;
  _ready_for_next_iteration_type ready_for_next_iteration;
  using _is_moving_type =
    bool;
  _is_moving_type is_moving;
  using _target_box_id_type =
    int32_t;
  _target_box_id_type target_box_id;
  using _iteration_type =
    int32_t;
  _iteration_type iteration;
  using _detection_done_type =
    bool;
  _detection_done_type detection_done;
  using _icp_done_type =
    bool;
  _icp_done_type icp_done;
  using _octomap_done_type =
    bool;
  _octomap_done_type octomap_done;
  using _nbv_done_type =
    bool;
  _nbv_done_type nbv_done;
  using _nbv_point_x_type =
    double;
  _nbv_point_x_type nbv_point_x;
  using _nbv_point_y_type =
    double;
  _nbv_point_y_type nbv_point_y;
  using _nbv_point_z_type =
    double;
  _nbv_point_z_type nbv_point_z;
  using _is_final_result_type =
    bool;
  _is_final_result_type is_final_result;

  // setters for named parameter idiom
  Type & set__ready_for_next_iteration(
    const bool & _arg)
  {
    this->ready_for_next_iteration = _arg;
    return *this;
  }
  Type & set__is_moving(
    const bool & _arg)
  {
    this->is_moving = _arg;
    return *this;
  }
  Type & set__target_box_id(
    const int32_t & _arg)
  {
    this->target_box_id = _arg;
    return *this;
  }
  Type & set__iteration(
    const int32_t & _arg)
  {
    this->iteration = _arg;
    return *this;
  }
  Type & set__detection_done(
    const bool & _arg)
  {
    this->detection_done = _arg;
    return *this;
  }
  Type & set__icp_done(
    const bool & _arg)
  {
    this->icp_done = _arg;
    return *this;
  }
  Type & set__octomap_done(
    const bool & _arg)
  {
    this->octomap_done = _arg;
    return *this;
  }
  Type & set__nbv_done(
    const bool & _arg)
  {
    this->nbv_done = _arg;
    return *this;
  }
  Type & set__nbv_point_x(
    const double & _arg)
  {
    this->nbv_point_x = _arg;
    return *this;
  }
  Type & set__nbv_point_y(
    const double & _arg)
  {
    this->nbv_point_y = _arg;
    return *this;
  }
  Type & set__nbv_point_z(
    const double & _arg)
  {
    this->nbv_point_z = _arg;
    return *this;
  }
  Type & set__is_final_result(
    const bool & _arg)
  {
    this->is_final_result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    message_interfaces::msg::NodeStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const message_interfaces::msg::NodeStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<message_interfaces::msg::NodeStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<message_interfaces::msg::NodeStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      message_interfaces::msg::NodeStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<message_interfaces::msg::NodeStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      message_interfaces::msg::NodeStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<message_interfaces::msg::NodeStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<message_interfaces::msg::NodeStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<message_interfaces::msg::NodeStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__message_interfaces__msg__NodeStatus
    std::shared_ptr<message_interfaces::msg::NodeStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__message_interfaces__msg__NodeStatus
    std::shared_ptr<message_interfaces::msg::NodeStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NodeStatus_ & other) const
  {
    if (this->ready_for_next_iteration != other.ready_for_next_iteration) {
      return false;
    }
    if (this->is_moving != other.is_moving) {
      return false;
    }
    if (this->target_box_id != other.target_box_id) {
      return false;
    }
    if (this->iteration != other.iteration) {
      return false;
    }
    if (this->detection_done != other.detection_done) {
      return false;
    }
    if (this->icp_done != other.icp_done) {
      return false;
    }
    if (this->octomap_done != other.octomap_done) {
      return false;
    }
    if (this->nbv_done != other.nbv_done) {
      return false;
    }
    if (this->nbv_point_x != other.nbv_point_x) {
      return false;
    }
    if (this->nbv_point_y != other.nbv_point_y) {
      return false;
    }
    if (this->nbv_point_z != other.nbv_point_z) {
      return false;
    }
    if (this->is_final_result != other.is_final_result) {
      return false;
    }
    return true;
  }
  bool operator!=(const NodeStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NodeStatus_

// alias to use template instance with default allocator
using NodeStatus =
  message_interfaces::msg::NodeStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace message_interfaces

#endif  // MESSAGE_INTERFACES__MSG__DETAIL__NODE_STATUS__STRUCT_HPP_
