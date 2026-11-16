#include <fstream>
#include <iterator>
#include <sstream>

#include "actions_common.h"

using namespace ueds_connector;

namespace
{

std::string Triple(const std::array<double, 3>& v) {
  return "(" + Num(v[0]) + ", " + Num(v[1]) + ", " + Num(v[2]) + ")";
}

void AddTextLines(Ctx& c, const std::string& text, size_t max_lines = 2000) {
  std::istringstream in(text);
  std::string        line;
  size_t             n = 0;
  while (std::getline(in, line)) {
    if (++n > max_lines) {
      c.line("... (truncated)");
      break;
    }
    c.line(line);
  }
}

}  // namespace

ActionList SceneActions(App& app) {
  ActionList   list;
  const Target GM = Target::GameMode;

  /* Objects //{ */

  {
    struct P
    {
      std::string path = "scene.yaml";
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Objects", "SpawnObjectsFromYaml", GM, "Read a spawn config on this machine and send it; assets resolve on the sim side.",
                                           [p](Ctx& c) {
                                             std::ifstream file(p->path);
                                             if (!file) {
                                               c.line("cannot open '" + p->path + "'");
                                               return false;
                                             }
                                             const std::string yaml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
                                             std::string error;
                                             auto [ok, ids] = c.gm->SpawnObjectsFromYaml(yaml, error);
                                             if (!ok) {
                                               c.line("error: " + error);
                                               return false;
                                             }
                                             std::string s = std::to_string(ids.size()) + " object(s), ids:";
                                             for (int id : ids) {
                                               s += " " + std::to_string(id);
                                             }
                                             c.line(s);
                                             return true;
                                           }));
    Bind(a, p, {Row("YAML path", {Field::S(&p->path, "", 50)})});
  }

  list.push_back(MakeAction("Objects", "ListSpawnedObjects", GM, "Every spawned object with its current pose (ROS frame, m / deg).", [](Ctx& c) {
    auto [ok, objects] = c.gm->ListSpawnedObjects();
    if (!ok) {
      return false;
    }
    c.line(std::to_string(objects.size()) + " object(s)");
    for (const auto& o : objects) {
      c.line("  #" + std::to_string(o.id) + "  " + o.asset + "  stencil " + std::to_string(o.stencil) + "  pos " + Triple(o.position) + "  rpy " +
             Triple(o.orientation) + "  scale " + Triple(o.scale));
    }
    return true;
  }));

  {
    struct P
    {
      int                   id = 0;
      std::array<double, 3> pos{0, 0, 0}, rpy{0, 0, 0}, scale{1, 1, 1};
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Objects", "MoveSpawnedObject", GM, "Reposition a spawned object. 'Load current' fetches its pose by id.",
                                           [p](Ctx& c) { return c.gm->MoveSpawnedObject(p->id, p->pos, p->rpy, p->scale); }));
    a.load = [p](Ctx& c) {
      auto [ok, objects] = c.gm->ListSpawnedObjects();
      if (!ok) {
        return false;
      }
      for (const auto& o : objects) {
        if (o.id == p->id) {
          p->pos   = o.position;
          p->rpy   = o.orientation;
          p->scale = o.scale;
          c.line("#" + std::to_string(o.id) + " " + o.asset + " loaded");
          return true;
        }
      }
      c.line("no object #" + std::to_string(p->id));
      return false;
    };
    Bind(a, p,
         {Row("Object id", {Field::I(&p->id)}), Row("Position [m]", Vec3(&p->pos[0], &p->pos[1], &p->pos[2])),
          Row("Orientation [deg]", Vec3(&p->rpy[0], &p->rpy[1], &p->rpy[2], "roll", "pitch", "yaw")),
          Row("Scale", Vec3(&p->scale[0], &p->scale[1], &p->scale[2]))});
  }

  {
    struct P
    {
      int id = 0;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Objects", "RemoveSpawnedObject", GM, "Remove one spawned object (negative id removes all).", [p](Ctx& c) {
      auto [ok, count] = c.gm->RemoveSpawnedObject(p->id);
      c.line("removed " + std::to_string(count));
      return ok;
    }));
    Bind(a, p, {Row("Object id", {Field::I(&p->id)})});
  }

  list.push_back(MakeAction("Objects", "RemoveAllSpawnedObjects", GM, "Remove everything the spawn tool placed.", [](Ctx& c) {
    auto [ok, count] = c.gm->RemoveAllSpawnedObjects();
    c.line("removed " + std::to_string(count));
    return ok;
  }));

  {
    struct P
    {
      std::string save_path;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Objects", "ExportScene", GM, "Spawned objects as a spawn config; optionally saved to a file.", [p](Ctx& c) {
      auto [ok, yaml] = c.gm->ExportScene();
      if (!ok) {
        return false;
      }
      if (!p->save_path.empty()) {
        std::ofstream out(p->save_path);
        out << yaml;
        c.line(out ? "saved to " + p->save_path : "could not write " + p->save_path);
      } else {
        c.line(std::to_string(yaml.size()) + " bytes");
      }
      AddTextLines(c, yaml);
      return true;
    }));
    Bind(a, p, {Row("Save to (optional)", {Field::S(&p->save_path, "", 50)})});
  }

  {
    struct P
    {
      std::string prefix, filter;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Objects", "ListAssets", GM, "What the spawn tool can place: cooked assets under a prefix plus on-disk glTF.",
                                           [p](Ctx& c) {
                                             bool truncated  = false;
                                             auto [ok, assets] = c.gm->ListAssets(p->prefix, p->filter, truncated);
                                             if (!ok) {
                                               return false;
                                             }
                                             c.line(std::to_string(assets.size()) + " asset(s)" + (truncated ? " (truncated by server cap)" : ""));
                                             for (const auto& s : assets) {
                                               c.line("  " + s);
                                             }
                                             return true;
                                           }));
    Bind(a, p, {Row("Path prefix", {Field::S(&p->prefix, "", 40)}, "empty = /Game"), Row("Name filter", {Field::S(&p->filter, "", 30)})});
  }

  //}

  /* Geometry //{ */

  {
    struct P
    {
      std::array<double, 3> center{0, 0, 0}, extent{0, 0, 0};
      int                   max_triangles = 200000;
      std::string           save_path;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Geometry", "GetWorldMesh", GM, "Static world geometry as a triangle soup (scene frame, m); optional PLY export.",
                                           [p](Ctx& c) {
                                             std::vector<float>    vertices;
                                             std::vector<uint32_t> indices;
                                             bool                  truncated = false;
                                             int32_t               sources   = 0;
                                             const bool ok = c.gm->GetWorldMesh(p->center, p->extent, static_cast<uint32_t>(std::max(0, p->max_triangles)), vertices,
                                                                                indices, truncated, sources);
                                             if (!ok) {
                                               return false;
                                             }
                                             c.line(std::to_string(vertices.size() / 3) + " vertices, " + std::to_string(indices.size() / 3) + " triangles from " +
                                                    std::to_string(sources) + " source(s)" + (truncated ? ", truncated" : ""));
                                             if (!p->save_path.empty()) {
                                               std::ofstream out(p->save_path);
                                               out << "ply\nformat ascii 1.0\nelement vertex " << vertices.size() / 3
                                                   << "\nproperty float x\nproperty float y\nproperty float z\nelement face " << indices.size() / 3
                                                   << "\nproperty list uchar int vertex_indices\nend_header\n";
                                               for (size_t i = 0; i + 2 < vertices.size(); i += 3) {
                                                 out << vertices[i] << ' ' << vertices[i + 1] << ' ' << vertices[i + 2] << '\n';
                                               }
                                               for (size_t i = 0; i + 2 < indices.size(); i += 3) {
                                                 out << "3 " << indices[i] << ' ' << indices[i + 1] << ' ' << indices[i + 2] << '\n';
                                               }
                                               c.line(out ? "saved " + p->save_path : "could not write " + p->save_path);
                                             }
                                             return true;
                                           }));
    Bind(a, p,
         {Row("Center [m]", Vec3(&p->center[0], &p->center[1], &p->center[2])), Row("Extent [m]", Vec3(&p->extent[0], &p->extent[1], &p->extent[2]), "0 = everything"),
          Row("Max triangles", {Field::I(&p->max_triangles, "", 10)}, "0 = no cap"), Row("Save PLY (optional)", {Field::S(&p->save_path, "", 40)})});
  }

  {
    struct P
    {
      std::string points       = "0 0 1; 5 0 1";
      double      max_radius   = 5.0;
      double      tolerance    = 0.001;
      bool        terrain_only = false;
    };
    auto p = std::make_shared<P>();
    auto& a = list.emplace_back(MakeAction("Geometry", "GetDistanceBatch", GM, "Exact distance to the nearest collision surface for points 'x y z; x y z' (m).",
                                           [p](Ctx& c) {
                                             std::vector<float> query;
                                             std::string        normalized = p->points;
                                             std::replace(normalized.begin(), normalized.end(), ';', ' ');
                                             std::replace(normalized.begin(), normalized.end(), ',', ' ');
                                             std::istringstream in(normalized);
                                             float              v;
                                             while (in >> v) {
                                               query.push_back(v);
                                             }
                                             if (query.empty() || query.size() % 3 != 0) {
                                               c.line("need groups of three numbers, got " + std::to_string(query.size()));
                                               return false;
                                             }
                                             std::vector<float> distances;
                                             const bool ok = c.gm->GetDistanceBatch(query, p->max_radius, p->tolerance, p->terrain_only, distances);
                                             if (!ok) {
                                               return false;
                                             }
                                             for (size_t i = 0; i < distances.size(); ++i) {
                                               c.line("(" + Num(query[i * 3]) + ", " + Num(query[i * 3 + 1]) + ", " + Num(query[i * 3 + 2]) + ") → " + Num(distances[i], 4) +
                                                      " m");
                                             }
                                             return true;
                                           }));
    Bind(a, p,
         {Row("Points", {Field::S(&p->points, "", 50)}), Row("Max radius [m]", {Field::D(&p->max_radius)}), Row("Tolerance [m]", {Field::D(&p->tolerance)}),
          Row("Terrain only", {Field::B(&p->terrain_only)})});
  }

  //}

  (void)app;
  return list;
}
