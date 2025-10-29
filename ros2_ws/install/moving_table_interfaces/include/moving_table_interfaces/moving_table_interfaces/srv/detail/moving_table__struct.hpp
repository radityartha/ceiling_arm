// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from moving_table_interfaces:srv/MovingTable.idl
// generated code does not contain a copyright notice

#ifndef MOVING_TABLE_INTERFACES__SRV__DETAIL__MOVING_TABLE__STRUCT_HPP_
#define MOVING_TABLE_INTERFACES__SRV__DETAIL__MOVING_TABLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__moving_table_interfaces__srv__MovingTable_Request __attribute__((deprecated))
#else
# define DEPRECATED__moving_table_interfaces__srv__MovingTable_Request __declspec(deprecated)
#endif

namespace moving_table_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MovingTable_Request_
{
  using Type = MovingTable_Request_<ContainerAllocator>;

  explicit MovingTable_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->table_id = "";
      this->distance_mm = 0.0f;
      this->angle_deg = 0.0f;
      this->linear_speed = 0l;
      this->rotate_speed = 0l;
      this->operation_type = 0l;
    }
  }

  explicit MovingTable_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : table_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->table_id = "";
      this->distance_mm = 0.0f;
      this->angle_deg = 0.0f;
      this->linear_speed = 0l;
      this->rotate_speed = 0l;
      this->operation_type = 0l;
    }
  }

  // field types and members
  using _table_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _table_id_type table_id;
  using _distance_mm_type =
    float;
  _distance_mm_type distance_mm;
  using _angle_deg_type =
    float;
  _angle_deg_type angle_deg;
  using _linear_speed_type =
    int32_t;
  _linear_speed_type linear_speed;
  using _rotate_speed_type =
    int32_t;
  _rotate_speed_type rotate_speed;
  using _operation_type_type =
    int32_t;
  _operation_type_type operation_type;

  // setters for named parameter idiom
  Type & set__table_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->table_id = _arg;
    return *this;
  }
  Type & set__distance_mm(
    const float & _arg)
  {
    this->distance_mm = _arg;
    return *this;
  }
  Type & set__angle_deg(
    const float & _arg)
  {
    this->angle_deg = _arg;
    return *this;
  }
  Type & set__linear_speed(
    const int32_t & _arg)
  {
    this->linear_speed = _arg;
    return *this;
  }
  Type & set__rotate_speed(
    const int32_t & _arg)
  {
    this->rotate_speed = _arg;
    return *this;
  }
  Type & set__operation_type(
    const int32_t & _arg)
  {
    this->operation_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    moving_table_interfaces::srv::MovingTable_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const moving_table_interfaces::srv::MovingTable_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<moving_table_interfaces::srv::MovingTable_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<moving_table_interfaces::srv::MovingTable_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      moving_table_interfaces::srv::MovingTable_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<moving_table_interfaces::srv::MovingTable_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      moving_table_interfaces::srv::MovingTable_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<moving_table_interfaces::srv::MovingTable_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<moving_table_interfaces::srv::MovingTable_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<moving_table_interfaces::srv::MovingTable_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__moving_table_interfaces__srv__MovingTable_Request
    std::shared_ptr<moving_table_interfaces::srv::MovingTable_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__moving_table_interfaces__srv__MovingTable_Request
    std::shared_ptr<moving_table_interfaces::srv::MovingTable_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MovingTable_Request_ & other) const
  {
    if (this->table_id != other.table_id) {
      return false;
    }
    if (this->distance_mm != other.distance_mm) {
      return false;
    }
    if (this->angle_deg != other.angle_deg) {
      return false;
    }
    if (this->linear_speed != other.linear_speed) {
      return false;
    }
    if (this->rotate_speed != other.rotate_speed) {
      return false;
    }
    if (this->operation_type != other.operation_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const MovingTable_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MovingTable_Request_

// alias to use template instance with default allocator
using MovingTable_Request =
  moving_table_interfaces::srv::MovingTable_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace moving_table_interfaces


#ifndef _WIN32
# define DEPRECATED__moving_table_interfaces__srv__MovingTable_Response __attribute__((deprecated))
#else
# define DEPRECATED__moving_table_interfaces__srv__MovingTable_Response __declspec(deprecated)
#endif

namespace moving_table_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MovingTable_Response_
{
  using Type = MovingTable_Response_<ContainerAllocator>;

  explicit MovingTable_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit MovingTable_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    moving_table_interfaces::srv::MovingTable_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const moving_table_interfaces::srv::MovingTable_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<moving_table_interfaces::srv::MovingTable_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<moving_table_interfaces::srv::MovingTable_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      moving_table_interfaces::srv::MovingTable_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<moving_table_interfaces::srv::MovingTable_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      moving_table_interfaces::srv::MovingTable_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<moving_table_interfaces::srv::MovingTable_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<moving_table_interfaces::srv::MovingTable_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<moving_table_interfaces::srv::MovingTable_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__moving_table_interfaces__srv__MovingTable_Response
    std::shared_ptr<moving_table_interfaces::srv::MovingTable_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__moving_table_interfaces__srv__MovingTable_Response
    std::shared_ptr<moving_table_interfaces::srv::MovingTable_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MovingTable_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const MovingTable_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MovingTable_Response_

// alias to use template instance with default allocator
using MovingTable_Response =
  moving_table_interfaces::srv::MovingTable_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace moving_table_interfaces

namespace moving_table_interfaces
{

namespace srv
{

struct MovingTable
{
  using Request = moving_table_interfaces::srv::MovingTable_Request;
  using Response = moving_table_interfaces::srv::MovingTable_Response;
};

}  // namespace srv

}  // namespace moving_table_interfaces

#endif  // MOVING_TABLE_INTERFACES__SRV__DETAIL__MOVING_TABLE__STRUCT_HPP_
