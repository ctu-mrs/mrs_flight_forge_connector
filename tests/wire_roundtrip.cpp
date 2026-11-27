#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include <flight_forge_connector/wire/flightforge_generated.h>

#include "test_framework.h"

namespace wire = flightforge::wire;

namespace
{

/* helpers //{ */

wire::Pose MakePose(const double X, const double Y, const double Z, const double Yaw)
{
  return wire::Pose(wire::Vec3(X, Y, Z), wire::Quat(0.0, 0.0, std::sin(Yaw / 2.0), std::cos(Yaw / 2.0)));
}

bool VerifyRequest(const flatbuffers::FlatBufferBuilder& Builder)
{
  flatbuffers::Verifier Verifier(Builder.GetBufferPointer(), Builder.GetSize());
  return wire::VerifyRequestBuffer(Verifier);
}

bool VerifyResponse(const uint8_t* Data, const size_t Size)
{
  flatbuffers::Verifier Verifier(Data, Size);
  return Verifier.VerifyBuffer<wire::Response>(nullptr);
}

//}

/* every request body //{ */

FF_TEST(TestEveryRequestBody)
{
  int Built = 0;

  for (auto Type = static_cast<int>(wire::RequestBody::MIN) + 1; Type <= static_cast<int>(wire::RequestBody::MAX); ++Type) {
    const auto Body = static_cast<wire::RequestBody>(Type);

    flatbuffers::FlatBufferBuilder Builder;
    const auto Empty = Builder.EndTable(Builder.StartTable());
    wire::RequestBuilder Envelope(Builder);
    Envelope.add_seq(static_cast<uint32_t>(Type));
    Envelope.add_robot_id(3);
    Envelope.add_body_type(Body);
    Envelope.add_body(flatbuffers::Offset<void>(Empty));
    Builder.Finish(Envelope.Finish());

    const std::string Name = wire::EnumNameRequestBody(Body);
    FF_REQUIRE(VerifyRequest(Builder));

    const wire::Request* Read = wire::GetRequest(Builder.GetBufferPointer());
    FF_CHECK(Read->seq() == static_cast<uint32_t>(Type));
    FF_CHECK(Read->body_type() == Body);
    FF_CHECK(Read->robot_id().has_value() && *Read->robot_id() == 3);
    FF_CHECK(!Read->sensor_id().has_value());
    FF_CHECK(Name.size() > 7 && Name.compare(Name.size() - 7, 7, "Request") == 0);

    const auto Unpacked = wire::UnPackRequest(Builder.GetBufferPointer());
    FF_REQUIRE(Unpacked->body.type == Body && Unpacked->body.value != nullptr);

    flatbuffers::FlatBufferBuilder Again;
    wire::FinishRequestBuffer(Again, wire::Request::Pack(Again, Unpacked.get()));
    FF_REQUIRE(VerifyRequest(Again));
    FF_CHECK(*wire::UnPackRequest(Again.GetBufferPointer()) == *Unpacked);
    ++Built;
  }

  FF_CHECK(Built == static_cast<int>(wire::RequestBody::MAX));
}

//}

/* set_states //{ */

FF_TEST(TestSetStates)
{
  constexpr int Robots = 5;

  wire::SetStatesRequestT States;
  States.collide      = true;
  States.wait_applied = false;
  for (int Id = 0; Id < Robots; ++Id) {
    auto Entry      = std::make_unique<wire::RobotStateEntryT>();
    Entry->robot_id = Id;
    Entry->state    = std::make_unique<wire::RobotStateT>();
    Entry->state->base = std::make_unique<wire::Pose>(MakePose(Id, -Id, 2.5, 0.1 * Id));
    Entry->state->joint_velocities = {1000.0 + Id, -1000.0 - Id, 1000.0, -1000.0};
    if (Id % 2 == 0) {
      Entry->state->joint_positions = {0.25 * Id};
      Entry->state->link_poses      = {MakePose(0.1, 0.2, 0.3, 0.0), MakePose(-0.1, 0.2, 0.3, M_PI)};
    }
    States.states.push_back(std::move(Entry));
  }

  wire::RequestT Envelope;
  Envelope.seq = 77;
  Envelope.body.Set(std::move(States));

  flatbuffers::FlatBufferBuilder Builder;
  wire::FinishRequestBuffer(Builder, wire::Request::Pack(Builder, &Envelope));
  FF_REQUIRE(VerifyRequest(Builder));

  const wire::Request* Read = wire::GetRequest(Builder.GetBufferPointer());
  FF_CHECK(!Read->robot_id().has_value());
  const wire::SetStatesRequest* Body = Read->body_as_SetStatesRequest();
  FF_REQUIRE(Body != nullptr && Body->states() != nullptr && Body->states()->size() == Robots);
  FF_CHECK(Body->collide() && !Body->wait_applied());

  for (int Id = 0; Id < Robots; ++Id) {
    const wire::RobotStateEntry* Entry = Body->states()->Get(Id);
    const wire::Pose*            Base  = Entry->state()->base();
    FF_CHECK(Entry->robot_id() == Id);
    FF_REQUIRE(Base != nullptr);
    FF_CHECK(Base->position().x() == Id && Base->position().y() == -Id && Base->position().z() == 2.5);
    FF_CHECK_NEAR(Base->orientation().z(), std::sin(0.05 * Id), 1e-15);
    FF_CHECK(Entry->state()->joint_velocities()->size() == 4 && Entry->state()->joint_velocities()->Get(1) == -1000.0 - Id);
    if (Id % 2 == 0) {
      FF_CHECK(Entry->state()->link_poses()->size() == 2);
      FF_CHECK_NEAR(Entry->state()->link_poses()->Get(1)->orientation().z(), 1.0, 1e-15);
    } else {
      FF_CHECK(Entry->state()->link_poses() == nullptr && Entry->state()->joint_positions() == nullptr);
    }
  }

  FF_CHECK(*wire::UnPackRequest(Builder.GetBufferPointer()) == Envelope);
}

//}

/* lidar scan //{ */

FF_TEST(TestLidarScan)
{
  constexpr uint32_t Rows = 16;
  constexpr uint32_t Cols = 64;
  constexpr uint32_t N    = Rows * Cols;
  const float        NaN  = std::numeric_limits<float>::quiet_NaN();

  flatbuffers::FlatBufferBuilder Builder;

  wire::Vec3f* Points = nullptr;
  const auto   PointsOffset = Builder.CreateUninitializedVectorOfStructs<wire::Vec3f>(N, &Points);
  for (uint32_t I = 0; I < N; ++I) {
    Points[I] = I % 7 == 0 ? wire::Vec3f(NaN, NaN, NaN) : wire::Vec3f(0.01f * I, -0.02f * I, 0.5f);
  }

  std::vector<int32_t>  Intensity(N);
  std::vector<int32_t>  ClassId(N);
  std::vector<uint32_t> InstanceId(N);
  for (uint32_t I = 0; I < N; ++I) {
    const bool Return = I % 7 != 0;
    Intensity[I]      = Return ? static_cast<int32_t>(I % 5) : -1;
    ClassId[I]        = Return ? static_cast<int32_t>(I % 11) : -1;
    InstanceId[I]     = Return ? I : 0;
  }
  const auto IntensityOffset  = Builder.CreateVector(Intensity);
  const auto ClassIdOffset    = Builder.CreateVector(ClassId);
  const auto InstanceIdOffset = Builder.CreateVector(InstanceId);

  const wire::Pose SensorPose = MakePose(10.0, -4.0, 1.5, M_PI / 2.0);
  const auto Scan = wire::CreateLidarScanResponse(Builder, 12.25, &SensorPose, Rows, Cols, PointsOffset, IntensityOffset, ClassIdOffset, InstanceIdOffset);
  Builder.Finish(wire::CreateResponse(Builder, 9, true, 0, wire::ResponseBody::LidarScanResponse, Scan.Union()));
  FF_REQUIRE(VerifyResponse(Builder.GetBufferPointer(), Builder.GetSize()));

  const auto* Read = flatbuffers::GetRoot<wire::Response>(Builder.GetBufferPointer())->body_as_LidarScanResponse();
  FF_REQUIRE(Read != nullptr && Read->points() != nullptr);
  FF_CHECK(Read->stamp() == 12.25 && Read->rows() == Rows && Read->cols() == Cols);
  FF_CHECK(Read->sensor_pose()->position().x() == 10.0 && Read->sensor_pose()->orientation().w() == SensorPose.orientation().w());
  FF_REQUIRE(Read->points()->size() == N && Read->intensity()->size() == N && Read->class_id()->size() == N && Read->instance_id()->size() == N);

  int NaNs = 0;
  bool Match = true;
  for (uint32_t I = 0; I < N; ++I) {
    const wire::Vec3f* P = Read->points()->Get(I);
    if (I % 7 == 0) {
      NaNs += std::isnan(P->x()) && std::isnan(P->y()) && std::isnan(P->z());
      Match = Match && Read->intensity()->Get(I) == -1 && Read->class_id()->Get(I) == -1 && Read->instance_id()->Get(I) == 0;
    } else {
      Match = Match && P->x() == 0.01f * I && P->y() == -0.02f * I && P->z() == 0.5f;
      Match = Match && Read->intensity()->Get(I) == Intensity[I] && Read->class_id()->Get(I) == ClassId[I] && Read->instance_id()->Get(I) == I;
    }
  }
  FF_CHECK(NaNs == static_cast<int>((N + 6) / 7));
  FF_CHECK(Match);
  FF_CHECK(sizeof(wire::Vec3f) == 12);
}

//}

/* camera frame //{ */

FF_TEST(TestCameraFrame)
{
  constexpr uint32_t Width  = 1024;
  constexpr uint32_t Height = 256;
  constexpr uint32_t Bytes  = Width * Height * 4;

  flatbuffers::FlatBufferBuilder Builder(Bytes + 1024);

  uint8_t*   Pixels = nullptr;
  const auto Data   = Builder.CreateUninitializedVector<uint8_t>(Bytes, &Pixels);
  for (uint32_t I = 0; I < Bytes; ++I) {
    Pixels[I] = static_cast<uint8_t>(I * 31 + (I >> 12));
  }
  const auto Image = wire::CreateImage(Builder, Width, Height, wire::PixelFormat::BGRA8, wire::Compression::NONE, Data);
  const auto Frame = wire::CreateRgbCameraDataResponse(Builder, 3.5, Image);
  Builder.Finish(wire::CreateResponse(Builder, 1, true, 0, wire::ResponseBody::RgbCameraDataResponse, Frame.Union()));
  FF_REQUIRE(VerifyResponse(Builder.GetBufferPointer(), Builder.GetSize()));
  FF_CHECK(Builder.GetSize() < Bytes + 256);

  const uint8_t*   Buffer = Builder.GetBufferPointer();
  const auto*      Read   = flatbuffers::GetRoot<wire::Response>(Buffer)->body_as_RgbCameraDataResponse();
  FF_REQUIRE(Read != nullptr && Read->image() != nullptr && Read->image()->data() != nullptr);
  const auto* ReadData = Read->image()->data();
  FF_CHECK(Read->stamp() == 3.5 && Read->image()->width() == Width && Read->image()->height() == Height);
  FF_CHECK(Read->image()->format() == wire::PixelFormat::BGRA8 && Read->image()->compression() == wire::Compression::NONE);
  FF_REQUIRE(ReadData->size() == Bytes);
  FF_CHECK(ReadData->data() >= Buffer && ReadData->data() + Bytes <= Buffer + Builder.GetSize());

  bool Match = true;
  for (uint32_t I = 0; I < Bytes; ++I) {
    Match = Match && ReadData->Get(I) == static_cast<uint8_t>(I * 31 + (I >> 12));
  }
  FF_CHECK(Match);

  std::vector<uint8_t> Truncated(Buffer, Buffer + Builder.GetSize() / 2);
  FF_CHECK(!VerifyResponse(Truncated.data(), Truncated.size()));
}

//}

/* get_info //{ */

FF_TEST(TestGetInfo)
{
  std::ifstream        File(FF_WIRE_BFBS, std::ios::binary);
  std::vector<uint8_t> Schema((std::istreambuf_iterator<char>(File)), std::istreambuf_iterator<char>());
  FF_REQUIRE(Schema.size() > 1024);
  FF_CHECK(flatbuffers::BufferHasIdentifier(Schema.data(), "BFBS"));

  flatbuffers::FlatBufferBuilder Builder;
  wire::FinishRequestBuffer(Builder, wire::CreateRequest(Builder, 1, flatbuffers::nullopt, flatbuffers::nullopt, wire::RequestBody::GetInfoRequest,
                                                         wire::CreateGetInfoRequest(Builder).Union()));
  FF_REQUIRE(VerifyRequest(Builder));
  FF_CHECK(wire::GetRequest(Builder.GetBufferPointer())->body_as_GetInfoRequest() != nullptr);

  flatbuffers::FlatBufferBuilder Reply;
  const auto Info = wire::CreateGetInfoResponseDirect(Reply, 1, 0, &Schema);
  Reply.Finish(wire::CreateResponse(Reply, 1, true, 0, wire::ResponseBody::GetInfoResponse, Info.Union()));
  FF_REQUIRE(VerifyResponse(Reply.GetBufferPointer(), Reply.GetSize()));

  const auto* Read = flatbuffers::GetRoot<wire::Response>(Reply.GetBufferPointer());
  FF_CHECK(Read->ok() && Read->seq() == 1 && Read->error() == nullptr);
  const wire::GetInfoResponse* Body = Read->body_as_GetInfoResponse();
  FF_REQUIRE(Body != nullptr && Body->schema() != nullptr);
  FF_CHECK(Body->api_major() == 1 && Body->api_minor() == 0);
  FF_CHECK(Body->schema()->size() == Schema.size() && std::memcmp(Body->schema()->data(), Schema.data(), Schema.size()) == 0);
}

//}

}  // namespace

int main()
{
  return fftest::RunAll();
}
