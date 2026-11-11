#pragma once

#include <cstring>
#include <string>

#include <flight_forge_connector/connection.h>
#include <flight_forge_connector/wire_convert.h>

namespace flightforge
{

inline Status AsStatus(const Reply& reply) {
  return reply.ok ? Status::Ok() : Status::Fail(reply.error.empty() ? "the request failed" : reply.error);
}

template <typename T>
const T* Body(const Reply& reply, Status& status) {
  if (!reply.ok) {
    status = AsStatus(reply);
    return nullptr;
  }
  const T* body = reply.body<T>();
  if (!body) {
    status = Status::Fail(std::string("the reply carries no ") + wire::EnumNameResponseBody(wire::ResponseBodyTraits<T>::enum_value));
  }
  return body;
}

inline SetStateResult ReadStateResult(const wire::SetStateResponse* result) {
  SetStateResult out;
  out.robot_id = result->robot_id();
  out.hit      = result->hit();
  if (result->base()) {
    out.base = FromWire(*result->base());
  }
  if (result->impact_point()) {
    out.impact_point = FromWire(*result->impact_point());
  }
  return out;
}

}  // namespace flightforge
