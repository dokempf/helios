#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include <Color4f.h>
#include <Material.h>
#include <Vertex.h>

/**
 * @brief Bulk storage for triangle primitives.
 *
 * Vertices are stored per-triangle (3 * n). Indices are implicit by order.
 */
struct TriangleBulk
{
  std::vector<Vertex> vertices;
  std::vector<glm::dvec3> face_normal;
  std::vector<glm::dvec3> e1;
  std::vector<glm::dvec3> e2;
  std::vector<glm::dvec3> v0;
  std::vector<double> eps;
  std::vector<glm::dvec3> aabb_min;
  std::vector<glm::dvec3> aabb_max;
  std::vector<std::shared_ptr<Material>> materials;

  inline void clear()
  {
    vertices.clear();
    face_normal.clear();
    e1.clear();
    e2.clear();
    v0.clear();
    eps.clear();
    aabb_min.clear();
    aabb_max.clear();
    materials.clear();
  }

  inline std::size_t size() const { return face_normal.size(); }
};

/**
 * @brief Bulk storage for voxel primitives.
 */
struct VoxelBulk
{
  std::vector<Vertex> centers;
  std::vector<double> half_size;
  std::vector<int> num_points;
  std::vector<double> r;
  std::vector<double> g;
  std::vector<double> b;
  std::vector<Color4f> color;
  std::vector<glm::dvec3> aabb_min;
  std::vector<glm::dvec3> aabb_max;
  std::vector<std::shared_ptr<Material>> materials;

  inline void clear()
  {
    centers.clear();
    half_size.clear();
    num_points.clear();
    r.clear();
    g.clear();
    b.clear();
    color.clear();
    aabb_min.clear();
    aabb_max.clear();
    materials.clear();
  }

  inline std::size_t size() const { return centers.size(); }
};

/**
 * @brief Bulk storage for DetailedVoxel extra data.
 */
struct DetailedVoxelBulk
{
  std::vector<std::uint8_t> present;
  std::vector<std::vector<int>> int_values;
  std::vector<std::vector<double>> double_values;
  std::vector<double> max_pad;

  inline void clear()
  {
    present.clear();
    int_values.clear();
    double_values.clear();
    max_pad.clear();
  }

  inline std::size_t size() const { return present.size(); }
};

using PrimitiveIndex = std::size_t;
