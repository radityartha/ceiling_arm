// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from moving_table_interfaces:srv/MovingTable.idl
// generated code does not contain a copyright notice

#ifndef MOVING_TABLE_INTERFACES__SRV__DETAIL__MOVING_TABLE__TRAITS_HPP_
#define MOVING_TABLE_INTERFACES__SRV__DETAIL__MOVING_TABLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "moving_table_interfaces/srv/detail/moving_table__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace moving_table_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MovingTable_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: table_id
  {
    out << "table_id: ";
    rosidl_generator_traits::value_to_yaml(msg.table_id, out);
    out << ", ";
  }

  // member: distance_mm
  {
    out << "distance_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_mm, out);
    out << ", ";
  }

  // member: angle_deg
  {
    out << "angle_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_deg, out);
    out << ", ";
  }

  // member: linear_speed
  {
    out << "linear_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_speed, out);
    out << ", ";
  }

  // member: rotate_speed
  {
    out << "rotate_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.rotate_speed, out);
    out << ", ";
  }

  // member: operation_type
  {
    out << "operation_type: ";
    rosidl_generator_traits::value_to_yaml(msg.operation_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MovingTable_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: table_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "table_id: ";
    rosidl_generator_traits::value_to_yaml(msg.table_id, out);
    out << "\n";
  }

  // member: distance_mm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_mm: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_mm, out);
    out << "\n";
  }

  // member: angle_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_deg, out);
    out << "\n";
  }

  // member: linear_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_speed, out);
    out << "\n";
  }

  // member: rotate_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rotate_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.rotate_speed, out);
    out << "\n";
  }

  // member: operation_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operation_type: ";
    rosidl_generator_traits::value_to_yaml(msg.operation_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MovingTable_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace moving_table_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use moving_table_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const moving_table_interfaces::srv::MovingTable_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  moving_table_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use moving_table_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const moving_table_interfaces::srv::MovingTable_Request & msg)
{
  return moving_table_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<moving_table_interfaces::srv::MovingTable_Request>()
{
  return "moving_table_interfaces::srv::MovingTable_Request";
}

template<>
inline const char * name<moving_table_interfaces::srv::MovingTable_Request>()
{
  return "moving_table_interfaces/srv/MovingTable_Request";
}

template<>
struct has_fixed_size<moving_table_interfaces::srv::MovingTable_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<moving_table_interfaces::srv::MovingTable_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<moving_table_interfaces::srv::MovingTable_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace moving_table_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MovingTable_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MovingTable_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MovingTable_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace moving_table_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use moving_table_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const moving_table_interfaces::srv::MovingTable_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  moving_table_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use moving_table_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const moving_table_interfaces::srv::MovingTable_Response & msg)
{
  return moving_table_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<moving_table_interfaces::srv::MovingTable_Response>()
{
  return "moving_table_interfaces::srv::MovingTable_Response";
}

template<>
inline const char * name<moving_table_interfaces::srv::MovingTable_Response>()
{
  return "moving_table_interfaces/srv/MovingTable_Response";
}

template<>
struct has_fixed_size<moving_table_interfaces::srv::MovingTable_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<moving_table_interfaces::srv::MovingTable_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<moving_table_interfaces::srv::MovingTable_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<moving_table_interfaces::srv::MovingTable>()
{
  return "moving_table_interfaces::srv::MovingTable";
}

template<>
inline const char * name<moving_table_interfaces::srv::MovingTable>()
{
  return "moving_table_interfaces/srv/MovingTable";
}

template<>
struct has_fixed_size<moving_table_interfaces::srv::MovingTable>
  : std::integral_constant<
    bool,
    has_fixed_size<moving_table_interfaces::srv::MovingTable_Request>::value &&
    has_fixed_size<moving_table_interfaces::srv::MovingTable_Response>::value
  >
{
};

template<>
struct has_bounded_size<moving_table_interfaces::srv::MovingTable>
  : std::integral_constant<
    bool,
    has_bounded_size<moving_table_interfaces::srv::MovingTable_Request>::value &&
    has_bounded_size<moving_table_interfaces::srv::MovingTable_Response>::value
  >
{
};

template<>
struct is_service<moving_table_interfaces::srv::MovingTable>
  : std::true_type
{
};

template<>
struct is_service_request<moving_table_interfaces::srv::MovingTable_Request>
  : std::true_type
{
};

template<>
struct is_service_response<moving_table_interfaces::srv::MovingTable_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MOVING_TABLE_INTERFACES__SRV__DETAIL__MOVING_TABLE__TRAITS_HPP_
