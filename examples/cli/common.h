#pragma once

#include <cstdlib>
#include <string>

#include <flight_forge_connector/connection.h>

namespace ffcli
{

inline uint16_t Port() {
  const char* env = std::getenv("FLIGHTFORGE_PORT");
  return env ? static_cast<uint16_t>(std::atoi(env)) : flightforge::Connection::kDefaultPort;
}

inline std::string Address() {
  const char* env = std::getenv("FLIGHTFORGE_ADDRESS");
  return env ? env : "127.0.0.1";
}

}  // namespace ffcli
