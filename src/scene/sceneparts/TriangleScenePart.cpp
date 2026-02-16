#include <TriangleScenePart.h>

#include <AABB.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
inline glm::dvec3
loadVec3(arma::mat const& m, std::size_t row, std::size_t col0)
{
  return glm::dvec3(m(row, col0), m(row, col0 + 1), m(row, col0 + 2));
}

inline void
storeVec3(arma::mat& m, std::size_t row, std::size_t col0, glm::dvec3 const& v)
{
  m(row, col0) = v.x;
  m(row, col0 + 1) = v.y;
  m(row, col0 + 2) = v.z;
}
}

TriangleScenePart::TriangleScenePart()
{
  geometryType = GeometryType::TRIANGLE;
}

TriangleScenePart::TriangleScenePart(std::size_t triangleCount)
  : TriangleScenePart()
{
  vertices.set_size(triangleCount, 9);
  normals.zeros(triangleCount, 9);
  materialIndex.zeros(triangleCount);
}

std::shared_ptr<ScenePart>
TriangleScenePart::clone(bool) const
{
  std::shared_ptr<TriangleScenePart> out =
    std::make_shared<TriangleScenePart>(*this);
  out->materialTable.clear();
  out->materialTable.reserve(materialTable.size());
  for (std::shared_ptr<Material> const& material : materialTable) {
    if (material == nullptr) {
      out->materialTable.push_back(nullptr);
    } else {
      out->materialTable.push_back(std::make_shared<Material>(*material));
    }
  }
  return out;
}

std::size_t
TriangleScenePart::geometryCount() const
{
  return vertices.n_rows;
}

ScenePart::GeometryType
TriangleScenePart::geometryTypeOf() const
{
  return GeometryType::TRIANGLE;
}

std::shared_ptr<AABB>
TriangleScenePart::computeBound()
{
  if (vertices.n_elem == 0) {
    bound = nullptr;
    return bound;
  }
  double minX = std::numeric_limits<double>::max();
  double minY = std::numeric_limits<double>::max();
  double minZ = std::numeric_limits<double>::max();
  double maxX = std::numeric_limits<double>::lowest();
  double maxY = std::numeric_limits<double>::lowest();
  double maxZ = std::numeric_limits<double>::lowest();

  for (std::size_t i = 0; i < vertices.n_rows; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      glm::dvec3 const v = loadVec3(vertices, i, j * 3);
      minX = std::min(minX, v.x);
      minY = std::min(minY, v.y);
      minZ = std::min(minZ, v.z);
      maxX = std::max(maxX, v.x);
      maxY = std::max(maxY, v.y);
      maxZ = std::max(maxZ, v.z);
    }
  }
  bound = std::make_shared<AABB>(glm::dvec3(minX, minY, minZ),
                                 glm::dvec3(maxX, maxY, maxZ));
  centroid = arma::colvec(
    { (minX + maxX) * 0.5, (minY + maxY) * 0.5, (minZ + maxZ) * 0.5 });
  return bound;
}

glm::dvec3
TriangleScenePart::geometryCentroid(std::size_t index) const
{
  if (index >= vertices.n_rows) {
    return glm::dvec3(0.0);
  }
  glm::dvec3 const v0 = loadVec3(vertices, index, 0);
  glm::dvec3 const v1 = loadVec3(vertices, index, 3);
  glm::dvec3 const v2 = loadVec3(vertices, index, 6);
  return (v0 + v1 + v2) / 3.0;
}

double
TriangleScenePart::geometryRayIntersectionDistance(
  std::size_t index,
  glm::dvec3 const& rayOrigin,
  glm::dvec3 const& rayDir) const
{
  std::vector<double> const t =
    geometryRayIntersection(index, rayOrigin, rayDir);
  return t.empty() ? -1.0 : t[0];
}

std::vector<double>
TriangleScenePart::geometryRayIntersection(std::size_t index,
                                           glm::dvec3 const& rayOrigin,
                                           glm::dvec3 const& rayDir) const
{
  if (index >= vertices.n_rows) {
    return {};
  }
  glm::dvec3 const v0 = loadVec3(vertices, index, 0);
  glm::dvec3 const v1 = loadVec3(vertices, index, 3);
  glm::dvec3 const v2 = loadVec3(vertices, index, 6);
  glm::dvec3 const e1 = v1 - v0;
  glm::dvec3 const e2 = v2 - v0;
  glm::dvec3 const p = glm::cross(rayDir, e2);
  double const det = glm::dot(e1, p);
  if (std::abs(det) < 1e-12) {
    return {};
  }
  double const invDet = 1.0 / det;
  glm::dvec3 const tvec = rayOrigin - v0;
  double const u = glm::dot(tvec, p) * invDet;
  if (u < 0.0 || u > 1.0) {
    return {};
  }
  glm::dvec3 const q = glm::cross(tvec, e1);
  double const v = glm::dot(rayDir, q) * invDet;
  if (v < 0.0 || (u + v) > 1.0) {
    return {};
  }
  double const t = glm::dot(e2, q) * invDet;
  if (t < 0.0) {
    return {};
  }
  return { t };
}

double
TriangleScenePart::geometryIncidenceAngle(std::size_t index,
                                          glm::dvec3 const&,
                                          glm::dvec3 const& rayDir,
                                          glm::dvec3 const&) const
{
  if (index >= vertices.n_rows) {
    return 0.0;
  }
  glm::dvec3 const v0 = loadVec3(vertices, index, 0);
  glm::dvec3 const v1 = loadVec3(vertices, index, 3);
  glm::dvec3 const v2 = loadVec3(vertices, index, 6);
  glm::dvec3 const normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
  // Keep incidence angle orientation-insensitive to match legacy behavior.
  double const c =
    glm::clamp(std::abs(glm::dot(-glm::normalize(rayDir), normal)), -1.0, 1.0);
  return std::acos(c);
}

std::shared_ptr<AABB>
TriangleScenePart::geometryAABB(std::size_t index) const
{
  if (index >= vertices.n_rows) {
    return nullptr;
  }

  glm::dvec3 const v0 = loadVec3(vertices, index, 0);
  glm::dvec3 const v1 = loadVec3(vertices, index, 3);
  glm::dvec3 const v2 = loadVec3(vertices, index, 6);
  glm::dvec3 const mn(std::min({ v0.x, v1.x, v2.x }),
                      std::min({ v0.y, v1.y, v2.y }),
                      std::min({ v0.z, v1.z, v2.z }));
  glm::dvec3 const mx(std::max({ v0.x, v1.x, v2.x }),
                      std::max({ v0.y, v1.y, v2.y }),
                      std::max({ v0.z, v1.z, v2.z }));
  return std::make_shared<AABB>(mn, mx);
}

void
TriangleScenePart::geometryRotate(std::size_t index, Rotation& rotation)
{
  if (index >= vertices.n_rows) {
    return;
  }
  for (std::size_t j = 0; j < 3; ++j) {
    glm::dvec3 const v = rotation.applyTo(loadVec3(vertices, index, j * 3));
    storeVec3(vertices, index, j * 3, v);
  }
  if (normals.n_rows > index) {
    for (std::size_t j = 0; j < 3; ++j) {
      glm::dvec3 const n = rotation.applyTo(loadVec3(normals, index, j * 3));
      storeVec3(normals, index, j * 3, n);
    }
  }
}

void
TriangleScenePart::geometryScale(std::size_t index, double factor)
{
  if (index >= vertices.n_rows) {
    return;
  }
  for (std::size_t j = 0; j < 3; ++j) {
    glm::dvec3 const v = loadVec3(vertices, index, j * 3) * factor;
    storeVec3(vertices, index, j * 3, v);
  }
}

void
TriangleScenePart::geometryTranslate(std::size_t index, glm::dvec3 const& shift)
{
  if (index >= vertices.n_rows) {
    return;
  }
  for (std::size_t j = 0; j < 3; ++j) {
    glm::dvec3 const v = loadVec3(vertices, index, j * 3) + shift;
    storeVec3(vertices, index, j * 3, v);
  }
}

void
TriangleScenePart::geometryUpdate(std::size_t)
{
  // No cache to update for the current bulk representation.
}

void
TriangleScenePart::deleteGeometries()
{
  clearGeometryStorage();
}

void
TriangleScenePart::clearGeometryStorage()
{
  vertices.reset();
  normals.reset();
  materialIndex.reset();
  materialTable.clear();
  centroid.reset();
  bound = nullptr;
}

std::size_t
TriangleScenePart::geometryVertexCount(std::size_t index) const
{
  return index < vertices.n_rows ? 3 : 0;
}

std::size_t
TriangleScenePart::geometryDynamicVertexCount(std::size_t index) const
{
  return geometryVertexCount(index);
}

glm::dvec3
TriangleScenePart::geometryDynamicVertexPosition(std::size_t geometryIndex,
                                                 std::size_t vertexIndex) const
{
  if (geometryIndex >= vertices.n_rows || vertexIndex >= 3) {
    return glm::dvec3(0.0);
  }
  return loadVec3(vertices, geometryIndex, vertexIndex * 3);
}

glm::dvec3
TriangleScenePart::geometryDynamicVertexNormal(std::size_t geometryIndex,
                                               std::size_t vertexIndex) const
{
  if (geometryIndex >= normals.n_rows || vertexIndex >= 3) {
    return glm::dvec3(0.0);
  }
  return loadVec3(normals, geometryIndex, vertexIndex * 3);
}

void
TriangleScenePart::setGeometryDynamicVertexPosition(std::size_t geometryIndex,
                                                    std::size_t vertexIndex,
                                                    glm::dvec3 const& position)
{
  if (geometryIndex >= vertices.n_rows || vertexIndex >= 3) {
    return;
  }
  storeVec3(vertices, geometryIndex, vertexIndex * 3, position);
}

void
TriangleScenePart::setGeometryDynamicVertexNormal(std::size_t geometryIndex,
                                                  std::size_t vertexIndex,
                                                  glm::dvec3 const& normal)
{
  if (geometryIndex >= normals.n_rows || vertexIndex >= 3) {
    return;
  }
  storeVec3(normals, geometryIndex, vertexIndex * 3, normal);
}

std::shared_ptr<Material>
TriangleScenePart::geometryMaterial(std::size_t index) const
{
  if (index >= materialIndex.n_elem) {
    return nullptr;
  }
  std::size_t const midx = materialIndex(index);
  if (midx >= materialTable.size()) {
    return nullptr;
  }
  return materialTable[midx];
}

void
TriangleScenePart::setGeometryMaterial(std::size_t index,
                                       std::shared_ptr<Material> material)
{
  if (index >= materialIndex.n_elem || material == nullptr) {
    return;
  }
  for (std::size_t i = 0; i < materialTable.size(); ++i) {
    if (materialTable[i] == material) {
      materialIndex(index) = i;
      return;
    }
  }
  materialTable.push_back(material);
  materialIndex(index) = materialTable.size() - 1;
}
