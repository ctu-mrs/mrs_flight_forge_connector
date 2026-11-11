#pragma once

#include <memory>
#include <flight_forge_connector/data_types.h>
#include <flight_forge_connector/serialization/serializable_shared.h>

namespace Serializable::GameMode::GetWorldOrigin
{
inline std::unique_ptr<flightforge::Coordinates> ResponseToCoordinates(std::unique_ptr<Response> response) {
  return std::make_unique<flightforge::Coordinates>(response->x, response->y, response->z);
}
}

// namespace Serializable::GameMode::SpawnDroneAtLocation
// {
// inline std::unique_ptr<Request> CoordinateToRequest(const flightforge::Coordinates& coordinate) {
//   auto request = std::make_unique<Request>();
//   request->x   = coordinate.x;
//   request->y   = coordinate.y;
//   request->z   = coordinate.z;
//   return request;
// }
// } 

namespace Serializable::Drone::GetLocation
{
inline std::unique_ptr<flightforge::Coordinates> ResponseToCoordinates(std::unique_ptr<Response> response) {
  return std::make_unique<flightforge::Coordinates>(response->x, response->y, response->z);
}
}  // namespace Serializable::Drone::GetLocation

namespace Serializable::Drone::SetLocation
{
inline std::unique_ptr<Request> CoordinateToRequest(const flightforge::Coordinates& coordinate) {
  auto request = std::make_unique<Request>();
  request->x   = coordinate.x;
  request->y   = coordinate.y;
  request->z   = coordinate.z;
  return request;
}
}  // namespace Serializable::Drone::SetLocation

namespace Serializable::Drone::GetRotation
{
inline std::unique_ptr<flightforge::Rotation> ResponseToRotation(std::unique_ptr<Response> response) {
  return std::make_unique<flightforge::Rotation>(response->pitch, response->yaw, response->roll);
}
}  // namespace Serializable::Drone::GetRotation

namespace Serializable::Drone::SetRotation
{
inline std::unique_ptr<Request> RotationToRequest(const flightforge::Rotation& rotation) {
  auto request   = std::make_unique<Request>();
  request->pitch = rotation.pitch;
  request->yaw   = rotation.yaw;
  request->roll  = rotation.roll;
  return request;
}
}  // namespace Serializable::Drone::SetRotation

namespace Serializable::Drone::SetLocationAndRotation
{
inline std::unique_ptr<Request> ToRequest(const flightforge::Coordinates& location, const flightforge::Rotation& rotation) {
  auto request   = std::make_unique<Request>();
  request->x     = location.x;
  request->y     = location.y;
  request->z     = location.z;
  request->pitch = rotation.pitch;
  request->yaw   = rotation.yaw;
  request->roll  = rotation.roll;
  return request;
}
}  // namespace Serializable::Drone::SetLocationAndRotation

namespace Serializable::Drone::SetLocationAndRotationAsync
{
inline std::unique_ptr<Request> ToRequest(const flightforge::Coordinates& location, const flightforge::Rotation& rotation) {
  auto request   = std::make_unique<Request>();
  request->x     = location.x;
  request->y     = location.y;
  request->z     = location.z;
  request->pitch = rotation.pitch;
  request->yaw   = rotation.yaw;
  request->roll  = rotation.roll;
  return request;
}
}  // namespace Serializable::Drone::SetLocationAndRotationAsync
