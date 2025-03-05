// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from message_interfaces:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef MESSAGE_INTERFACES__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_
#define MESSAGE_INTERFACES__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__message_interfaces__msg__BoundingBox __attribute__((deprecated))
#else
# define DEPRECATED__message_interfaces__msg__BoundingBox __declspec(deprecated)
#endif

namespace message_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBox_
{
  using Type = BoundingBox_<ContainerAllocator>;

  explicit BoundingBox_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lu_x = 0l;
      this->lu_y = 0l;
      this->rd_x = 0l;
      this->rd_y = 0l;
    }
  }

  explicit BoundingBox_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lu_x = 0l;
      this->lu_y = 0l;
      this->rd_x = 0l;
      this->rd_y = 0l;
    }
  }

  // field types and members
  using _lu_x_type =
    int32_t;
  _lu_x_type lu_x;
  using _lu_y_type =
    int32_t;
  _lu_y_type lu_y;
  using _rd_x_type =
    int32_t;
  _rd_x_type rd_x;
  using _rd_y_type =
    int32_t;
  _rd_y_type rd_y;

  // setters for named parameter idiom
  Type & set__lu_x(
    const int32_t & _arg)
  {
    this->lu_x = _arg;
    return *this;
  }
  Type & set__lu_y(
    const int32_t & _arg)
  {
    this->lu_y = _arg;
    return *this;
  }
  Type & set__rd_x(
    const int32_t & _arg)
  {
    this->rd_x = _arg;
    return *this;
  }
  Type & set__rd_y(
    const int32_t & _arg)
  {
    this->rd_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    message_interfaces::msg::BoundingBox_<ContainerAllocator> *;
  using ConstRawPtr =
    const message_interfaces::msg::BoundingBox_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<message_interfaces::msg::BoundingBox_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<message_interfaces::msg::BoundingBox_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      message_interfaces::msg::BoundingBox_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<message_interfaces::msg::BoundingBox_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      message_interfaces::msg::BoundingBox_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<message_interfaces::msg::BoundingBox_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<message_interfaces::msg::BoundingBox_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<message_interfaces::msg::BoundingBox_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__message_interfaces__msg__BoundingBox
    std::shared_ptr<message_interfaces::msg::BoundingBox_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__message_interfaces__msg__BoundingBox
    std::shared_ptr<message_interfaces::msg::BoundingBox_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBox_ & other) const
  {
    if (this->lu_x != other.lu_x) {
      return false;
    }
    if (this->lu_y != other.lu_y) {
      return false;
    }
    if (this->rd_x != other.rd_x) {
      return false;
    }
    if (this->rd_y != other.rd_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBox_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBox_

// alias to use template instance with default allocator
using BoundingBox =
  message_interfaces::msg::BoundingBox_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace message_interfaces

#endif  // MESSAGE_INTERFACES__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_
