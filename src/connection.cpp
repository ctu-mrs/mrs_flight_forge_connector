#include <flight_forge_connector/connection.h>

#include <cstring>
#include <thread>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using ssize_t = int;
#define FF_CLOSE closesocket
#define FF_POLL WSAPoll
#else
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#define FF_CLOSE ::close
#define FF_POLL ::poll
#endif

namespace flightforge
{

namespace
{

constexpr uint32_t kMaxReplyBytes     = 1u << 30;
constexpr size_t   kReceiveChunkBytes = 256 * 1024;

#ifdef _WIN32
struct FWinsock
{
  FWinsock() {
    WSADATA data;
    WSAStartup(MAKEWORD(2, 2), &data);
  }
  ~FWinsock() {
    WSACleanup();
  }
};
#endif

}  // namespace

/* lifetime //{ */

Connection::Connection(std::string address, const uint16_t port) : address_(std::move(address)), port_(port) {
#ifdef _WIN32
  static FWinsock winsock;
#endif
}

Connection::~Connection() {
  Disconnect();
}

bool Connection::Connect(const int attempts, const std::chrono::milliseconds retry_delay) {
  for (int attempt = 0; attempt < std::max(attempts, 1); ++attempt) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (socket_ >= 0 || OpenLocked()) {
        return true;
      }
    }
    if (attempt + 1 < attempts) {
      std::this_thread::sleep_for(retry_delay);
    }
  }
  return false;
}

void Connection::Disconnect() {
  std::lock_guard<std::mutex> lock(mutex_);
  CloseLocked();
}

bool Connection::Connected() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return socket_ >= 0;
}

void Connection::SetReceiveTimeout(const std::chrono::milliseconds timeout) {
  std::lock_guard<std::mutex> lock(mutex_);
  timeout_ = timeout;
}

std::chrono::milliseconds Connection::ReceiveTimeout() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return timeout_;
}

bool Connection::OpenLocked() {
  addrinfo hints{};
  hints.ai_family   = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  addrinfo* found = nullptr;
  if (getaddrinfo(address_.c_str(), std::to_string(port_).c_str(), &hints, &found) != 0 || !found) {
    return false;
  }

  const auto fd = ::socket(found->ai_family, found->ai_socktype, found->ai_protocol);
  if (fd < 0) {
    freeaddrinfo(found);
    return false;
  }

  if (::connect(fd, found->ai_addr, static_cast<int>(found->ai_addrlen)) != 0) {
    FF_CLOSE(fd);
    freeaddrinfo(found);
    return false;
  }
  freeaddrinfo(found);

  // request/reply is what Nagle punishes
  int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof(one));
  int buffer = 4 << 20;
  setsockopt(fd, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&buffer), sizeof(buffer));

  socket_   = static_cast<intptr_t>(fd);
  rx_.clear();
  rx_start_ = 0;
  return true;
}

void Connection::CloseLocked() {
  if (socket_ >= 0) {
    FF_CLOSE(static_cast<int>(socket_));
    socket_ = -1;
  }
  ReleaseReceiveBuffer();
}

void Connection::ReleaseReceiveBuffer() {
  if (rx_.capacity() > 4 * kReceiveChunkBytes) {
    std::vector<uint8_t>().swap(rx_);
  } else {
    rx_.clear();
  }
  rx_start_ = 0;
}

/* //} */

/* I/O //{ */

bool Connection::WriteAll(const uint8_t* data, size_t size) {
  while (size > 0) {
#ifdef _WIN32
    const ssize_t sent = ::send(static_cast<SOCKET>(socket_), reinterpret_cast<const char*>(data), static_cast<int>(size), 0);
#else
    const ssize_t sent = ::send(static_cast<int>(socket_), data, size, MSG_NOSIGNAL);
#endif
    if (sent <= 0) {
      return false;
    }
    data += sent;
    size -= static_cast<size_t>(sent);
  }
  return true;
}

bool Connection::ReadFrame(std::vector<uint8_t>& out, const std::chrono::steady_clock::time_point deadline, std::string& error) {
  for (;;) {
    const size_t available = rx_.size() - rx_start_;
    size_t       missing   = 0;
    if (available >= kFrameHeaderBytes) {
      const uint32_t length = DecodeFrameLength(rx_.data() + rx_start_);
      if (length > kMaxReplyBytes) {
        error = "the simulator sent a " + std::to_string(length) + " byte frame";
        CloseLocked();
        return false;
      }
      if (available >= kFrameHeaderBytes + length) {
        const uint8_t* begin = rx_.data() + rx_start_ + kFrameHeaderBytes;
        out.assign(begin, begin + length);
        rx_start_ += kFrameHeaderBytes + length;
        if (rx_start_ == rx_.size()) {
          ReleaseReceiveBuffer();
        }
        return true;
      }
      missing = kFrameHeaderBytes + length - available;
    }

    if (rx_start_ > 0 && rx_start_ == rx_.size()) {
      ReleaseReceiveBuffer();
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      error = "timed out waiting for the reply";
      return false;
    }

    pollfd poll_fd{};
#ifdef _WIN32
    poll_fd.fd = static_cast<SOCKET>(socket_);
#else
    poll_fd.fd = static_cast<int>(socket_);
#endif
    poll_fd.events = POLLIN;
    const int wait  = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count()) + 1;
    const int ready = FF_POLL(&poll_fd, 1, wait);
    if (ready < 0) {
      error = "poll failed";
      CloseLocked();
      return false;
    }
    if (ready == 0) {
      continue;
    }

    const size_t old  = rx_.size();
    // resize zero-fills, so it covers this frame, never the capacity a past large frame left behind
    const size_t want = std::max<size_t>(kReceiveChunkBytes, missing);
    rx_.resize(old + want);
#ifdef _WIN32
    const ssize_t got = ::recv(static_cast<SOCKET>(socket_), reinterpret_cast<char*>(rx_.data() + old), static_cast<int>(want), 0);
#else
    const ssize_t got = ::recv(static_cast<int>(socket_), rx_.data() + old, want, 0);
#endif
    if (got <= 0) {
      rx_.resize(old);
      error = "the simulator closed the connection";
      CloseLocked();
      return false;
    }
    rx_.resize(old + static_cast<size_t>(got));
  }
}

Reply Connection::Exchange(const uint8_t* data, const size_t size, const uint32_t seq, const std::chrono::milliseconds timeout) {
  Reply reply;

  if (socket_ < 0 && !OpenLocked()) {
    reply.error = "not connected to " + address_ + ":" + std::to_string(port_);
    return reply;
  }

  uint8_t header[kFrameHeaderBytes];
  EncodeFrameHeader(static_cast<uint32_t>(size), header);
  if (!WriteAll(header, sizeof(header)) || !WriteAll(data, size)) {
    CloseLocked();
    reply.error = "the request could not be sent";
    return reply;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  for (;;) {
    if (!ReadFrame(reply.frame, deadline, reply.error)) {
      return reply;
    }

    flatbuffers::Verifier verifier(reply.frame.data(), reply.frame.size());
    if (!verifier.VerifyBuffer<wire::Response>(nullptr)) {
      reply.error = "the simulator sent a reply that is not a flightforge.wire.Response";
      return reply;
    }

    const wire::Response* response = flatbuffers::GetRoot<wire::Response>(reply.frame.data());

    // a reply to a request that already timed out
    if (response->seq() != seq) {
      continue;
    }

    reply.response = response;
    reply.ok       = response->ok();
    reply.error    = response->error() ? response->error()->str() : std::string();
    return reply;
  }
}

Reply Connection::Send(flatbuffers::FlatBufferBuilder& builder, const wire::RequestBody type, const flatbuffers::Offset<void> body,
                       const std::optional<int32_t> robot_id, const std::optional<int32_t> sensor_id, const std::optional<std::chrono::milliseconds> timeout) {
  std::lock_guard<std::mutex> lock(mutex_);

  // 0 is what the simulator answers when it could not read the seq
  if (next_seq_ == 0) {
    ++next_seq_;
  }
  const uint32_t seq = next_seq_++;

  const auto robot  = robot_id ? flatbuffers::Optional<int32_t>(*robot_id) : flatbuffers::nullopt;
  const auto sensor = sensor_id ? flatbuffers::Optional<int32_t>(*sensor_id) : flatbuffers::nullopt;
  builder.Finish(wire::CreateRequest(builder, seq, robot, sensor, type, body));

  return Exchange(builder.GetBufferPointer(), builder.GetSize(), seq, timeout.value_or(timeout_));
}

Reply Connection::SendRaw(const uint8_t* data, const size_t size, const uint32_t expect_seq, const std::optional<std::chrono::milliseconds> timeout) {
  std::lock_guard<std::mutex> lock(mutex_);
  return Exchange(data, size, expect_seq, timeout.value_or(timeout_));
}

/* //} */

}  // namespace flightforge
