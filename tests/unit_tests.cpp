#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <flight_forge_connector/connection.h>
#include <flight_forge_connector/data_types.h>
#include <flight_forge_connector/simulator_client.h>

#include "test_framework.h"

using namespace flightforge;

namespace
{

/* fake simulator //{ */

// answers each request on one connection with whatever Script decides, over plain POSIX sockets
class FakeSimulator {
public:
  using Script = std::function<void(int fd, uint32_t seq, int index)>;

  explicit FakeSimulator(Script script) : script_(std::move(script)) {
    listen_ = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port        = 0;
    ::bind(listen_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    ::listen(listen_, 1);
    socklen_t len = sizeof(addr);
    ::getsockname(listen_, reinterpret_cast<sockaddr*>(&addr), &len);
    port_   = ntohs(addr.sin_port);
    thread_ = std::thread([this] { Serve(); });
  }

  ~FakeSimulator() {
    ::shutdown(listen_, SHUT_RDWR);
    ::close(listen_);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  uint16_t Port() const {
    return port_;
  }

  static void Reply(const int fd, const uint32_t seq, const bool ok, const size_t chunk = 0, const size_t padding = 0) {
    flatbuffers::FlatBufferBuilder b;
    const auto                     error = padding ? b.CreateString(std::string(padding, 'x')) : flatbuffers::Offset<flatbuffers::String>();
    b.Finish(wire::CreateResponse(b, seq, ok, error));
    std::vector<uint8_t> frame(kFrameHeaderBytes + b.GetSize());
    EncodeFrameHeader(b.GetSize(), frame.data());
    std::copy(b.GetBufferPointer(), b.GetBufferPointer() + b.GetSize(), frame.begin() + kFrameHeaderBytes);
    for (size_t sent = 0; sent < frame.size();) {
      const size_t n = chunk ? std::min(chunk, frame.size() - sent) : frame.size() - sent;
      ::send(fd, frame.data() + sent, n, MSG_NOSIGNAL);
      sent += n;
      if (chunk) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  }

private:
  static bool ReadAll(const int fd, uint8_t* data, size_t size) {
    while (size > 0) {
      const ssize_t got = ::recv(fd, data, size, 0);
      if (got <= 0) {
        return false;
      }
      data += got;
      size -= static_cast<size_t>(got);
    }
    return true;
  }

  void Serve() {
    const int fd = ::accept(listen_, nullptr, nullptr);
    if (fd < 0) {
      return;
    }
    for (int index = 0;; ++index) {
      uint8_t header[kFrameHeaderBytes];
      if (!ReadAll(fd, header, sizeof(header))) {
        break;
      }
      std::vector<uint8_t> body(DecodeFrameLength(header));
      if (!ReadAll(fd, body.data(), body.size())) {
        break;
      }
      script_(fd, flatbuffers::GetRoot<wire::Request>(body.data())->seq(), index);
    }
    ::close(fd);
  }

  Script      script_;
  int         listen_ = -1;
  uint16_t    port_   = 0;
  std::thread thread_;
};

/* //} */

}  // namespace

/* framing //{ */

FF_TEST(FrameHeaderIsLittleEndianWithZeroFlags)
{
  uint8_t header[kFrameHeaderBytes];
  EncodeFrameHeader(0x01020304u, header);
  FF_CHECK(header[0] == 0x04 && header[1] == 0x03 && header[2] == 0x02 && header[3] == 0x01);
  FF_CHECK(header[4] == 0 && header[5] == 0 && header[6] == 0 && header[7] == 0);
  FF_CHECK(DecodeFrameLength(header) == 0x01020304u);
}

// a reply that arrives after its caller gave up must not answer the next request
FF_TEST(StaleReplyIsSkipped)
{
  uint32_t first_seq = 0;

  FakeSimulator fake([&first_seq](const int fd, const uint32_t seq, const int index) {
    if (index == 0) {
      first_seq = seq;
      return;
    }
    FakeSimulator::Reply(fd, first_seq, false);
    FakeSimulator::Reply(fd, seq, true);
  });

  auto connection = std::make_shared<Connection>("127.0.0.1", fake.Port());
  FF_REQUIRE(connection->Connect());
  connection->SetReceiveTimeout(std::chrono::milliseconds(100));

  SimulatorClient sim(connection);
  FF_CHECK(!sim.Ping());
  FF_CHECK(sim.Ping().ok);
}

FF_TEST(SplitFrameIsReassembled)
{
  FakeSimulator fake([](const int fd, const uint32_t seq, int) { FakeSimulator::Reply(fd, seq, true, 3); });

  SimulatorClient sim(std::make_shared<Connection>("127.0.0.1", fake.Port()));
  FF_REQUIRE(sim.Connect());
  FF_CHECK(sim.Ping().ok);
  FF_CHECK(sim.Ping().ok);
}

// a timeout part-way through a frame keeps what arrived, so the stream stays in step
FF_TEST(TimeoutMidFrameKeepsTheStreamInStep)
{
  FakeSimulator fake([](const int fd, const uint32_t seq, const int index) {
    if (index == 0) {
      FakeSimulator::Reply(fd, seq, true, 1);
    } else {
      FakeSimulator::Reply(fd, seq, true);
    }
  });

  auto connection = std::make_shared<Connection>("127.0.0.1", fake.Port());
  FF_REQUIRE(connection->Connect());
  connection->SetReceiveTimeout(std::chrono::milliseconds(10));

  SimulatorClient sim(connection);
  FF_CHECK(!sim.Ping());

  connection->SetReceiveTimeout(std::chrono::milliseconds(2000));
  FF_CHECK(sim.Ping().ok);
}

FF_TEST(NoSimulatorFailsCleanly)
{
  Connection connection("127.0.0.1", 1);
  FF_CHECK(!connection.Connect(2, std::chrono::milliseconds(10)));

  SimulatorClient sim("127.0.0.1", 1);
  const Status status = sim.Ping();
  FF_CHECK(!status.ok && !status.error.empty());
}

// the receive buffer once zero-filled all the capacity a large reply left behind, on every later read
FF_TEST(SmallRepliesStayFastAfterALargeOne)
{
  FakeSimulator fake([](const int fd, const uint32_t seq, const int index) { FakeSimulator::Reply(fd, seq, true, 0, index == 0 ? 64u << 20 : 0); });

  auto connection = std::make_shared<Connection>("127.0.0.1", fake.Port());
  FF_REQUIRE(connection->Connect());
  SimulatorClient sim(connection);
  sim.Ping();

  std::vector<double> samples;
  for (int i = 0; i < 50; ++i) {
    const auto start = std::chrono::steady_clock::now();
    FF_CHECK(sim.Ping().ok);
    samples.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
  }
  std::sort(samples.begin(), samples.end());
  FF_CHECK(samples[samples.size() / 2] < 1.0);
}

/* //} */

/* geometry //{ */

FF_TEST(QuaternionYawFacesPlusY)
{
  const Quat q = Quat::FromYaw(M_PI / 2);
  const Vec3 f = q.Rotate(Vec3{1.0, 0.0, 0.0});
  FF_CHECK_NEAR(f.x, 0.0, 1e-12);
  FF_CHECK_NEAR(f.y, 1.0, 1e-12);
  FF_CHECK_NEAR(q.Yaw(), M_PI / 2, 1e-12);

  const Quat r  = Quat::FromRpy(0.1, -0.2, 0.3);
  const Vec3 v  = r.Rotate(Vec3{0.3, -0.4, 0.5});
  const Vec3 v0 = r.Conjugate().Rotate(v);
  FF_CHECK_NEAR(v0.x, 0.3, 1e-12);
  FF_CHECK_NEAR(v0.y, -0.4, 1e-12);
  FF_CHECK_NEAR(v0.z, 0.5, 1e-12);

  const Quat yaw2 = Quat::FromYaw(0.25) * Quat::FromYaw(0.5);
  FF_CHECK_NEAR(yaw2.Yaw(), 0.75, 1e-12);
}

/* //} */

/* frame + world catalogs //{ */

FF_TEST(UavFrameCatalog)
{
  const auto& Names = UavFrameType::KnownNames();

  FF_CHECK(!Names.empty());
  FF_CHECK(std::find(Names.begin(), Names.end(), "x500") != Names.end());
  FF_CHECK(std::find(Names.begin(), Names.end(), "t650") != Names.end());
}

FF_TEST(WorldNameCatalog)
{
  const auto& Levels = WorldName::Name2Level();

  FF_CHECK(!Levels.empty());
  for (const auto& [Name, Level] : Levels) {
    FF_CHECK(!Name.empty());
    FF_CHECK(!Level.empty());
  }

  for (const char* Legacy : {"eletric_towers", "electric_towers", "sprind_lab", "kayenta_mine", "mala_skala", "warehouse"}) {
    FF_CHECK(Levels.count(Legacy) == 1);
  }
  FF_CHECK(Levels.at("sprind_lab") == "sprind_lab");
}

/* //} */

/* instance id convention //{ */

FF_TEST(InstanceIdEncoding)
{
  const auto Decode = [](const uint8_t R, const uint8_t G, const uint8_t B) { return uint32_t(R) + (uint32_t(G) << 8) + (uint32_t(B) << 16); };

  FF_CHECK(Decode(42, 0, 0) == 42u);
  FF_CHECK(Decode(0, 1, 0) == 256u);
  FF_CHECK(Decode(1, 1, 1) == 65793u);
  FF_CHECK(Decode(255, 255, 255) == 16777215u);

  for (const uint32_t Id : {0u, 1u, 255u, 256u, 65793u, 16777215u}) {
    FF_CHECK(Decode(Id & 0xFF, (Id >> 8) & 0xFF, (Id >> 16) & 0xFF) == Id);
  }
}

/* //} */

/* ground-truth query logic //{ */

// the sphere-overlap search behind get_distance_batch, against a predicate whose answer is known
FF_TEST(SphereSearchConvergesOnTheSurface)
{
  const auto Search = [](const auto& Blocked, const double MaxRadius, const double Tolerance) {
    if (Blocked(0.001)) {
      return 0.0;
    }
    if (!Blocked(MaxRadius)) {
      return MaxRadius;
    }

    double Lo = 0.0, Hi = MaxRadius;
    while (Hi - Lo > Tolerance) {
      const double Mid = 0.5 * (Lo + Hi);
      (Blocked(Mid) ? Hi : Lo) = Mid;
    }
    return 0.5 * (Lo + Hi);
  };

  for (const double Truth : {0.05, 0.5, 1.0, 2.75, 4.999}) {
    const auto   Blocked = [Truth](const double Radius) { return Radius >= Truth; };
    const double Found   = Search(Blocked, 5.0, 0.001);
    FF_CHECK(std::fabs(Found - Truth) <= 0.001);
  }

  FF_CHECK(Search([](double) { return false; }, 5.0, 0.001) == 5.0);
  FF_CHECK(Search([](double) { return true; }, 5.0, 0.001) == 0.0);

  const auto Blocked = [](const double Radius) { return Radius >= 1.234567; };
  FF_CHECK(std::fabs(Search(Blocked, 5.0, 0.01) - 1.234567) <= 0.01);
}

/* //} */

/* event packing //{ */

FF_TEST(CameraEventsUnpack)
{
  CameraEvents Events;
  Events.xyp        = {5u | (7u << 16) | 0x80000000u, 345u | (259u << 16)};
  Events.t_us       = {0u, 999u};
  Events.base_stamp = 12.5;

  FF_REQUIRE(Events.size() == 2);

  const auto First = Events.at(0);
  FF_CHECK(First.x == 5 && First.y == 7 && First.polarity == 1);
  FF_CHECK_NEAR(First.stamp, 12.5, 1e-12);

  const auto Second = Events.at(1);
  FF_CHECK(Second.x == 345 && Second.y == 259 && Second.polarity == -1);
  FF_CHECK_NEAR(Second.stamp, 12.500999, 1e-9);

  FF_CHECK(Events.unpack().size() == 2);
}

/* //} */

int main()
{
  return fftest::RunAll();
}
