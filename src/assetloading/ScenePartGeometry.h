#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>

#include <filems/serialization/serial_glm.h>

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

  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & vertices;
    ar & face_normal;
    ar & e1;
    ar & e2;
    ar & v0;
    ar & eps;
    ar & aabb_min;
    ar & aabb_max;
    ar & materials;
  }

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

  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & centers;
    ar & half_size;
    ar & num_points;
    ar & r;
    ar & g;
    ar & b;
    ar & color;
    ar & aabb_min;
    ar & aabb_max;
    ar & materials;
  }

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

  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & present;
    ar & int_values;
    ar & double_values;
    ar & max_pad;
  }

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

inline void
appendTriangleBulk(TriangleBulk& bulk,
                   Vertex const& v0,
                   Vertex const& v1,
                   Vertex const& v2,
                   std::shared_ptr<Material> const& material)
{
  bulk.vertices.push_back(v0);
  bulk.vertices.push_back(v1);
  bulk.vertices.push_back(v2);

  glm::dvec3 base = v0.pos;
  glm::dvec3 e1 = v1.pos - base;
  glm::dvec3 e2 = v2.pos - base;
  glm::dvec3 normal = glm::cross(e1, e2);
  double const len = glm::length(normal);
  if (len > 0.0)
    normal = normal / len;

  bulk.v0.push_back(base);
  bulk.e1.push_back(e1);
  bulk.e2.push_back(e2);
  bulk.face_normal.push_back(normal);
  bulk.eps.push_back(0.0000001);

  double minX = std::min(std::min(v0.getX(), v1.getX()), v2.getX());
  double minY = std::min(std::min(v0.getY(), v1.getY()), v2.getY());
  double minZ = std::min(std::min(v0.getZ(), v1.getZ()), v2.getZ());
  double maxX = std::max(std::max(v0.getX(), v1.getX()), v2.getX());
  double maxY = std::max(std::max(v0.getY(), v1.getY()), v2.getY());
  double maxZ = std::max(std::max(v0.getZ(), v1.getZ()), v2.getZ());
  bulk.aabb_min.push_back(glm::dvec3(minX, minY, minZ));
  bulk.aabb_max.push_back(glm::dvec3(maxX, maxY, maxZ));

  bulk.materials.push_back(material);
}

inline void
appendVoxelBulk(VoxelBulk& bulk,
                Vertex const& center,
                double const halfSize,
                int const numPoints,
                double const r,
                double const g,
                double const b,
                Color4f const& color,
                std::shared_ptr<Material> const& material)
{
  bulk.centers.push_back(center);
  bulk.half_size.push_back(halfSize);
  bulk.num_points.push_back(numPoints);
  bulk.r.push_back(r);
  bulk.g.push_back(g);
  bulk.b.push_back(b);
  bulk.color.push_back(color);
  glm::dvec3 hs(halfSize, halfSize, halfSize);
  bulk.aabb_min.push_back(center.pos - hs);
  bulk.aabb_max.push_back(center.pos + hs);
  bulk.materials.push_back(material);
}
