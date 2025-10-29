// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from moving_table_interfaces:srv/MovingTable.idl
// generated code does not contain a copyright notice

#ifndef MOVING_TABLE_INTERFACES__SRV__DETAIL__MOVING_TABLE__BUILDER_HPP_
#define MOVING_TABLE_INTERFACES__SRV__DETAIL__MOVING_TABLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "moving_table_interfaces/srv/detail/moving_table__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace moving_table_interfaces
{

namespace srv
{

namespace builder
{

class Init_MovingTable_Request_operation_type
{
public:
  explicit Init_MovingTable_Request_operation_type(::moving_table_interfaces::srv::MovingTable_Request & msg)
  : msg_(msg)
  {}
  ::moving_table_interfaces::srv::MovingTable_Request operation_type(::moving_table_interfaces::srv::MovingTable_Request::_operation_type_type arg)
  {
    msg_.operation_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::moving_table_interfaces::srv::MovingTable_Request msg_;
};

class Init_MovingTable_Request_rotate_speed
{
public:
  explicit Init_MovingTable_Request_rotate_speed(::moving_table_interfaces::srv::MovingTable_Request & msg)
  : msg_(msg)
  {}
  Init_MovingTable_Request_operation_type rotate_speed(::moving_table_interfaces::srv::MovingTable_Request::_rotate_speed_type arg)
  {
    msg_.rotate_speed = std::move(arg);
    return Init_MovingTable_Request_operation_type(msg_);
  }

private:
  ::moving_table_interfaces::srv::MovingTable_Request msg_;
};

class Init_MovingTable_Request_linear_speed
{
public:
  explicit Init_MovingTable_Request_linear_speed(::moving_table_interfaces::srv::MovingTable_Request & msg)
  : msg_(msg)
  {}
  Init_MovingTable_Request_rotate_speed linear_speed(::moving_table_interfaces::srv::MovingTable_Request::_linear_speed_type arg)
  {
    msg_.linear_speed = std::move(arg);
    return Init_MovingTable_Request_rotate_speed(msg_);
  }

private:
  ::moving_table_interfaces::srv::MovingTable_Request msg_;
};

class Init_MovingTable_Request_angle_deg
{
public:
  explicit Init_MovingTable_Request_angle_deg(::moving_table_interfaces::srv::MovingTable_Request & msg)
  : msg_(msg)
  {}
  Init_MovingTable_Request_linear_speed angle_deg(::moving_table_interfaces::srv::MovingTable_Request::_angle_deg_type arg)
  {
    msg_.angle_deg = std::move(arg);
    return Init_MovingTable_Request_linear_speed(msg_);
  }

private:
  ::moving_table_interfaces::srv::MovingTable_Request msg_;
};

class Init_MovingTable_Request_distance_mm
{
public:
  explicit Init_MovingTable_Request_distance_mm(::moving_table_interfaces::srv::MovingTable_Request & msg)
  : msg_(msg)
  {}
  Init_MovingTable_Request_angle_deg distance_mm(::moving_table_interfaces::srv::MovingTable_Request::_distance_mm_type arg)
  {
    msg_.distance_mm = std::move(arg);
    return Init_MovingTable_Request_angle_deg(msg_);
  }

private:
  ::moving_table_interfaces::srv::MovingTable_Request msg_;
};

class Init_MovingTable_Request_table_id
{
public:
  Init_MovingTable_Request_table_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovingTable_Request_distance_mm table_id(::moving_table_interfaces::srv::MovingTable_Request::_table_id_type arg)
  {
    msg_.table_id = std::move(arg);
    return Init_MovingTable_Request_distance_mm(msg_);
  }

private:
  ::moving_table_interfaces::srv::MovingTable_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::moving_table_interfaces::srv::MovingTable_Request>()
{
  return moving_table_interfaces::srv::builder::Init_MovingTable_Request_table_id();
}

}  // namespace moving_table_interfaces


namespace moving_table_interfaces
{

namespace srv
{

namespace builder
{

class Init_MovingTable_Response_message
{
public:
  explicit Init_MovingTable_Response_message(::moving_table_interfaces::srv::MovingTable_Response & msg)
  : msg_(msg)
  {}
  ::moving_table_interfaces::srv::MovingTable_Response message(::moving_table_interfaces::srv::MovingTable_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::moving_table_interfaces::srv::MovingTable_Response msg_;
};

class Init_MovingTable_Response_success
{
public:
  Init_MovingTable_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovingTable_Response_message success(::moving_table_interfaces::srv::MovingTable_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MovingTable_Response_message(msg_);
  }

private:
  ::moving_table_interfaces::srv::MovingTable_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::moving_table_interfaces::srv::MovingTable_Response>()
{
  return moving_table_interfaces::srv::builder::Init_MovingTable_Response_success();
}

}  // namespace moving_table_interfaces

#endif  // MOVING_TABLE_INTERFACES__SRV__DETAIL__MOVING_TABLE__BUILDER_HPP_
