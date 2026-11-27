// Offline ground truth: the world mesh as PLY, exact distances for a batch of points; both block the simulator's game thread.
//   extract_gt mesh [--center x y z] [--extent x y z] [--max-tris N] [-o out.ply]
//   extract_gt dist points.bin [--max-radius M] [--tolerance M] [--terrain-only] [-o out.bin]
// Everything is in the ground frame; points.bin and the distances are packed float32.

#include <array>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <flight_forge_connector/simulator_client.h>

#include "common.h"

namespace
{

void Usage() {
  std::cout << "usage:\n"
            << "  extract_gt mesh [--center x y z] [--extent x y z] [--max-tris N] [-o out.ply]\n"
            << "  extract_gt dist points.bin [--max-radius M] [--tolerance M] [--terrain-only] [-o out.bin]\n"
            << "  an extent of 0 on any axis exports the whole world; both calls block the simulator\n";
}

bool ReadTriple(char** argv, const int argc, int& i, flightforge::Vec3& out) {
  if (i + 3 >= argc) {
    return false;
  }
  out.x = std::stod(argv[++i]);
  out.y = std::stod(argv[++i]);
  out.z = std::stod(argv[++i]);
  return true;
}

bool WritePly(const std::string& path, const flightforge::WorldMesh& mesh) {
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << "ply\nformat ascii 1.0\nelement vertex " << mesh.vertices.size() << "\nproperty float x\nproperty float y\nproperty float z\n"
      << "element face " << mesh.indices.size() / 3 << "\nproperty list uchar int vertex_indices\nend_header\n";
  for (const auto& v : mesh.vertices) {
    out << v[0] << " " << v[1] << " " << v[2] << "\n";
  }
  for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
    out << "3 " << mesh.indices[i] << " " << mesh.indices[i + 1] << " " << mesh.indices[i + 2] << "\n";
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    Usage();
    return 1;
  }

  flightforge::SimulatorClient sim(ffcli::Address(), ffcli::Port());
  if (!sim.Connect()) {
    std::cerr << "no simulator on " << ffcli::Address() << ":" << ffcli::Port() << std::endl;
    return 1;
  }
  sim.SharedConnection()->SetReceiveTimeout(std::chrono::minutes(10));

  const std::string mode = argv[1];

  if (mode == "mesh") {
    flightforge::Vec3 center, extent;
    uint32_t          max_tris = 0;
    std::string       output   = "world.ply";
    for (int i = 2; i < argc; ++i) {
      const std::string arg = argv[i];
      if ((arg == "--center" && !ReadTriple(argv, argc, i, center)) || (arg == "--extent" && !ReadTriple(argv, argc, i, extent))) {
        Usage();
        return 1;
      }
      if (arg == "--max-tris" && i + 1 < argc) {
        max_tris = static_cast<uint32_t>(std::stoul(argv[++i]));
      } else if (arg == "-o" && i + 1 < argc) {
        output = argv[++i];
      }
    }

    const auto mesh = sim.GetWorldMesh(center, extent, max_tris);
    if (!mesh.ok) {
      std::cerr << "get_world_mesh failed: " << mesh.error << std::endl;
      return 1;
    }
    std::cout << mesh->vertices.size() << " vertices, " << mesh->indices.size() / 3 << " triangles from " << mesh->source_count << " sources"
              << (mesh->truncated ? " (truncated)" : "") << std::endl;
    return WritePly(output, *mesh) ? 0 : 1;
  }

  if (mode == "dist" && argc >= 3) {
    double      max_radius = 5.0, tolerance = 0.001;
    bool        terrain    = false;
    std::string output     = "distances.bin";
    for (int i = 3; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--max-radius" && i + 1 < argc) {
        max_radius = std::stod(argv[++i]);
      } else if (arg == "--tolerance" && i + 1 < argc) {
        tolerance = std::stod(argv[++i]);
      } else if (arg == "--terrain-only") {
        terrain = true;
      } else if (arg == "-o" && i + 1 < argc) {
        output = argv[++i];
      }
    }

    std::ifstream                      in(argv[2], std::ios::binary);
    std::vector<std::array<float, 3>> points;
    std::array<float, 3>              p;
    while (in.read(reinterpret_cast<char*>(p.data()), sizeof(p))) {
      points.push_back(p);
    }

    const auto distances = sim.GetDistanceBatch(points, max_radius, tolerance, terrain);
    if (!distances.ok) {
      std::cerr << "get_distance_batch failed: " << distances.error << std::endl;
      return 1;
    }
    std::ofstream(output, std::ios::binary).write(reinterpret_cast<const char*>(distances->data()), distances->size() * sizeof(float));
    std::cout << distances->size() << " distances written to " << output << std::endl;
    return 0;
  }

  Usage();
  return 1;
}
