#pragma once

#include <flight_forge_connector/data_types.h>

namespace flightforge
{

inline wire::Vec3 ToWire(const Vec3& v) {
  return wire::Vec3(v.x, v.y, v.z);
}

inline Vec3 FromWire(const wire::Vec3& v) {
  return Vec3{v.x(), v.y(), v.z()};
}

inline wire::Quat ToWire(const Quat& q) {
  return wire::Quat(q.x, q.y, q.z, q.w);
}

inline Quat FromWire(const wire::Quat& q) {
  return Quat{q.x(), q.y(), q.z(), q.w()};
}

inline wire::Pose ToWire(const Pose& p) {
  return wire::Pose(ToWire(p.position), ToWire(p.orientation));
}

inline Pose FromWire(const wire::Pose& p) {
  return Pose{FromWire(p.position()), FromWire(p.orientation())};
}

inline flatbuffers::Offset<wire::RobotState> BuildState(flatbuffers::FlatBufferBuilder& builder, const RobotState& state) {
  using Doubles          = flatbuffers::Offset<flatbuffers::Vector<double>>;
  const Doubles positions  = state.joint_positions.empty() ? Doubles() : builder.CreateVector(state.joint_positions);
  const Doubles velocities = state.joint_velocities.empty() ? Doubles() : builder.CreateVector(state.joint_velocities);

  const std::optional<wire::Pose> base    = state.base ? std::optional<wire::Pose>(ToWire(*state.base)) : std::nullopt;
  const std::optional<wire::Vec3> linear  = state.base_linear_velocity ? std::optional<wire::Vec3>(ToWire(*state.base_linear_velocity)) : std::nullopt;
  const std::optional<wire::Vec3> angular = state.base_angular_velocity ? std::optional<wire::Vec3>(ToWire(*state.base_angular_velocity)) : std::nullopt;

  return wire::CreateRobotState(builder, base ? &*base : nullptr, positions, velocities, 0, linear ? &*linear : nullptr, angular ? &*angular : nullptr);
}

inline RobotState ReadState(const wire::RobotState* state) {
  RobotState out;
  if (!state) {
    return out;
  }
  if (state->base()) {
    out.base = FromWire(*state->base());
  }
  if (state->joint_positions()) {
    out.joint_positions.assign(state->joint_positions()->begin(), state->joint_positions()->end());
  }
  if (state->joint_velocities()) {
    out.joint_velocities.assign(state->joint_velocities()->begin(), state->joint_velocities()->end());
  }
  if (state->base_linear_velocity()) {
    out.base_linear_velocity = FromWire(*state->base_linear_velocity());
  }
  if (state->base_angular_velocity()) {
    out.base_angular_velocity = FromWire(*state->base_angular_velocity());
  }
  return out;
}

inline Image ReadImage(const wire::Image* image, const double stamp) {
  Image out;
  out.stamp = stamp;
  if (image) {
    out.width       = image->width();
    out.height      = image->height();
    out.format      = image->format();
    out.compression = image->compression();
    if (image->data()) {
      out.data.assign(image->data()->begin(), image->data()->end());
    }
  }
  return out;
}

}  // namespace flightforge
