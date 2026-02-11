// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/DVL.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__DVL__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__DVL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/dvl__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_DVL_covariance
{
public:
  explicit Init_DVL_covariance(::interfaces::msg::DVL & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::DVL covariance(::interfaces::msg::DVL::_covariance_type arg)
  {
    msg_.covariance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::DVL msg_;
};

class Init_DVL_velocity
{
public:
  explicit Init_DVL_velocity(::interfaces::msg::DVL & msg)
  : msg_(msg)
  {}
  Init_DVL_covariance velocity(::interfaces::msg::DVL::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_DVL_covariance(msg_);
  }

private:
  ::interfaces::msg::DVL msg_;
};

class Init_DVL_header
{
public:
  Init_DVL_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DVL_velocity header(::interfaces::msg::DVL::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DVL_velocity(msg_);
  }

private:
  ::interfaces::msg::DVL msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::DVL>()
{
  return interfaces::msg::builder::Init_DVL_header();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__DVL__BUILDER_HPP_
