/**
 * Offline ground-truth extraction: writes the world mesh as PLY and exact distances
 * for a batch of query points. Both calls block the simulator's game thread, so this
 * is meant to run between flights rather than during one.
 *
 *   extract_gt mesh  [--center x y z] [--extent x y z] [--max-tris N] [-o out.ply]
 *   extract_gt dist  points.bin [--max-radius M] [--tolerance M] [--terrain-only] [-o out.bin]
 *
 * Frames: everything is the scene frame the spawn API uses - right-handed metres
 * relative to the world origin, y positive to the left. That is NOT Unreal's frame;
 * Unreal is left-handed centimetres. The conversion is done server-side, so points
 * written here line up with spawn_objects poses and with the lidar's own output.
 *
 * points.bin is packed float32 x,y,z; distances.bin comes back packed float32 in the
 * same order, in metres, equal to max-radius where truncated and 0 inside geometry.
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <flight_forge_connector/simulator_client.h>

namespace
{

/* usage + argument helpers //{ */

void Usage() {
  std::cout << "usage:\n"
            << "  extract_gt mesh [--center x y z] [--extent x y z] [--max-tris N] [-o out.ply]\n"
            << "  extract_gt dist points.bin [--max-radius M] [--tolerance M] [--terrain-only] [-o out.bin]\n\n"
            << "  poses are the scene frame: right-handed metres relative to the world origin,\n"
            << "  the same frame spawn_objects and the lidar use. An extent of 0 on any axis\n"
            << "  disables the box filter and exports the whole world.\n\n"
            << "  both calls block the simulator while they run.\n";
}

bool ReadTriple(char** Argv, const int Argc, int& i, std::array<double, 3>& Out) {
  if (i + 3 >= Argc) {
    return false;
  }
  for (double& Value : Out) {
    Value = std::stod(Argv[++i]);
  }
  return true;
}

/* //} */

/* writePly() //{ */

bool WritePly(const std::string& Path, const std::vector<float>& Vertices, const std::vector<uint32_t>& Indices) {

  std::ofstream Out(Path);
  if (!Out) {
    return false;
  }

  Out << "ply\nformat ascii 1.0\n"
      << "element vertex " << Vertices.size() / 3 << "\n"
      << "property float x\nproperty float y\nproperty float z\n"
      << "element face " << Indices.size() / 3 << "\n"
      << "property list uchar int vertex_indices\nend_header\n";

  for (size_t i = 0; i + 2 < Vertices.size(); i += 3) {
    Out << Vertices[i] << " " << Vertices[i + 1] << " " << Vertices[i + 2] << "\n";
  }
  for (size_t i = 0; i + 2 < Indices.size(); i += 3) {
    Out << "3 " << Indices[i] << " " << Indices[i + 1] << " " << Indices[i + 2] << "\n";
  }

  return true;
}

/* //} */

}  // namespace

/* main() //{ */

int main(int argc, char** argv) {

  if (argc < 2) {
    Usage();
    return 1;
  }

  const std::string Mode = argv[1];

  std::array<double, 3> Center{0.0, 0.0, 0.0};
  std::array<double, 3> Extent{0.0, 0.0, 0.0};
  uint32_t              MaxTriangles = 0;
  double                MaxRadius    = 5.0;
  double                Tolerance    = 0.001;
  bool                  TerrainOnly  = false;
  std::string           PointsPath;
  std::string           OutPath;

  for (int i = 2; i < argc; ++i) {
    const std::string Arg = argv[i];

    if (Arg == "--center" && !ReadTriple(argv, argc, i, Center)) {
      std::cerr << "--center needs three numbers\n";
      return 1;
    } else if (Arg == "--extent" && !ReadTriple(argv, argc, i, Extent)) {
      std::cerr << "--extent needs three numbers\n";
      return 1;
    } else if (Arg == "--max-tris" && i + 1 < argc) {
      MaxTriangles = static_cast<uint32_t>(std::stoul(argv[++i]));
    } else if (Arg == "--max-radius" && i + 1 < argc) {
      MaxRadius = std::stod(argv[++i]);
    } else if (Arg == "--tolerance" && i + 1 < argc) {
      Tolerance = std::stod(argv[++i]);
    } else if (Arg == "--terrain-only") {
      TerrainOnly = true;
    } else if ((Arg == "-o" || Arg == "--output") && i + 1 < argc) {
      OutPath = argv[++i];
    } else if (Arg == "-h" || Arg == "--help") {
      Usage();
      return 0;
    } else if (Arg[0] != '-' && PointsPath.empty()) {
      PointsPath = Arg;
    }
  }

  flightforge::SimulatorClient Controller("127.0.0.1", 8551);
  if (!Controller.Connect()) {
    std::cerr << "no simulator on 127.0.0.1:8551\n";
    return 1;
  }

  const auto Start = std::chrono::steady_clock::now();
  const auto Elapsed = [&Start] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - Start).count();
  };

  if (Mode == "mesh") {

    std::vector<float>    Vertices;
    std::vector<uint32_t> Indices;
    bool                  Truncated = false;
    int32_t               Sources   = 0;

    if (!Controller.GetWorldMesh(Center, Extent, MaxTriangles, Vertices, Indices, Truncated, Sources)) {
      std::cerr << "the simulator rejected the mesh request\n";
      return 1;
    }

    const std::string Path = OutPath.empty() ? "world_mesh.ply" : OutPath;
    if (!WritePly(Path, Vertices, Indices)) {
      std::cerr << "could not write " << Path << "\n";
      return 1;
    }

    std::cout << Indices.size() / 3 << " triangles, " << Vertices.size() / 3 << " vertices, from " << Sources << " components in " << Elapsed() << " s\n"
              << "wrote " << Path << (Truncated ? "  (TRUNCATED: raise --max-tris or narrow --extent)" : "") << "\n";
    return 0;
  }

  if (Mode == "dist") {

    if (PointsPath.empty()) {
      std::cerr << "dist needs a points.bin of packed float32 xyz\n";
      return 1;
    }

    std::ifstream In(PointsPath, std::ios::binary | std::ios::ate);
    if (!In) {
      std::cerr << "could not read " << PointsPath << "\n";
      return 1;
    }

    const std::streamsize Bytes = In.tellg();
    In.seekg(0);

    if (Bytes <= 0 || Bytes % (3 * sizeof(float)) != 0) {
      std::cerr << PointsPath << " is not a whole number of float32 xyz triples\n";
      return 1;
    }

    std::vector<float> Points(Bytes / sizeof(float));
    In.read(reinterpret_cast<char*>(Points.data()), Bytes);

    std::vector<float> Distances;
    if (!Controller.GetDistanceBatch(Points, MaxRadius, Tolerance, TerrainOnly, Distances)) {
      std::cerr << "the simulator rejected the distance request\n";
      return 1;
    }

    const std::string Path = OutPath.empty() ? "distances.bin" : OutPath;
    std::ofstream     Out(Path, std::ios::binary);
    if (!Out) {
      std::cerr << "could not write " << Path << "\n";
      return 1;
    }
    Out.write(reinterpret_cast<const char*>(Distances.data()), Distances.size() * sizeof(float));

    double Min = MaxRadius, Max = 0.0, Sum = 0.0;
    size_t Truncated = 0, Inside = 0;
    for (const float Distance : Distances) {
      Min = std::min<double>(Min, Distance);
      Max = std::max<double>(Max, Distance);
      Sum += Distance;
      Truncated += Distance >= MaxRadius ? 1 : 0;
      Inside += Distance == 0.0f ? 1 : 0;
    }

    std::cout << Distances.size() << " points in " << Elapsed() << " s (" << size_t(Distances.size() / std::max(Elapsed(), 1e-9)) << " pts/s)\n"
              << "  distance min " << Min << " max " << Max << " mean " << Sum / std::max<size_t>(Distances.size(), 1) << " m\n"
              << "  " << Truncated << " truncated at max-radius, " << Inside << " inside geometry\n"
              << "wrote " << Path << "\n";
    return 0;
  }

  Usage();
  return 1;
}

/* //} */
