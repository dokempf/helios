#include <VoxelScenePart.h>

#include <AABB.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
inline glm::dvec3
loadVec3(arma::mat const& m, std::size_t row)
{
  return glm::dvec3(m(row, 0), m(row, 1), m(row, 2));
}

inline void
storeVec3(arma::mat& m, std::size_t row, glm::dvec3 const& v)
{
  m(row, 0) = v.x;
  m(row, 1) = v.y;
  m(row, 2) = v.z;
}
}

VoxelScenePart::VoxelScenePart()
{
  geometryType = GeometryType::VOXEL;
}

VoxelScenePart::VoxelScenePart(std::size_t voxelCount)
  : VoxelScenePart()
{
  centers.zeros(voxelCount, 3);
  halfSizes.zeros(voxelCount, 3);
  normals.zeros(voxelCount, 3);
  materialIndex.zeros(voxelCount);
}

std::shared_ptr<ScenePart>
VoxelScenePart::clone(bool) const
{
  std::shared_ptr<VoxelScenePart> out = std::make_shared<VoxelScenePart>(*this);
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
VoxelScenePart::geometryCount() const
{
  return centers.n_rows;
}

ScenePart::GeometryType
VoxelScenePart::geometryTypeOf() const
{
  return GeometryType::VOXEL;
}

std::shared_ptr<AABB>
VoxelScenePart::computeBound()
{
  if (centers.n_rows == 0) {
    bound = nullptr;
    return bound;
  }
  double minX = std::numeric_limits<double>::max();
  double minY = std::numeric_limits<double>::max();
  double minZ = std::numeric_limits<double>::max();
  double maxX = std::numeric_limits<double>::lowest();
  double maxY = std::numeric_limits<double>::lowest();
  double maxZ = std::numeric_limits<double>::lowest();
  for (std::size_t i = 0; i < centers.n_rows; ++i) {
    glm::dvec3 const c = loadVec3(centers, i);
    glm::dvec3 const h = loadVec3(halfSizes, i);
    glm::dvec3 const mn = c - h;
    glm::dvec3 const mx = c + h;
    minX = std::min(minX, mn.x);
    minY = std::min(minY, mn.y);
    minZ = std::min(minZ, mn.z);
    maxX = std::max(maxX, mx.x);
    maxY = std::max(maxY, mx.y);
    maxZ = std::max(maxZ, mx.z);
  }
  bound = std::make_shared<AABB>(glm::dvec3(minX, minY, minZ),
                                 glm::dvec3(maxX, maxY, maxZ));
  centroid = arma::colvec(
    { (minX + maxX) * 0.5, (minY + maxY) * 0.5, (minZ + maxZ) * 0.5 });
  return bound;
}

glm::dvec3
VoxelScenePart::geometryCentroid(std::size_t index) const
{
  if (index >= centers.n_rows) {
    return glm::dvec3(0.0);
  }
  return loadVec3(centers, index);
}

std::vector<double>
VoxelScenePart::geometryRayIntersection(std::size_t index,
                                        glm::dvec3 const& rayOrigin,
                                        glm::dvec3 const& rayDir) const
{
  if (index >= centers.n_rows) {
    return {};
  }
  glm::dvec3 const c = loadVec3(centers, index);
  glm::dvec3 const h = loadVec3(halfSizes, index);
  AABB box(c - h, c + h);
  return box.getRayIntersection(rayOrigin, rayDir);
}

double
VoxelScenePart::geometryRayIntersectionDistance(std::size_t index,
                                                glm::dvec3 const& rayOrigin,
                                                glm::dvec3 const& rayDir) const
{
  std::vector<double> const t =
    geometryRayIntersection(index, rayOrigin, rayDir);
  return t.empty() ? -1.0 : t[0];
}

double
VoxelScenePart::geometryIncidenceAngle(std::size_t index,
                                       glm::dvec3 const&,
                                       glm::dvec3 const& rayDir,
                                       glm::dvec3 const&) const
{
  if (index >= normals.n_rows) {
    return 0.0;
  }
  glm::dvec3 n = loadVec3(normals, index);
  if (glm::length(n) == 0.0) {
    return 0.0;
  }
  n = glm::normalize(n);
  double const c = glm::clamp(glm::dot(-glm::normalize(rayDir), n), -1.0, 1.0);
  return std::acos(c);
}

std::shared_ptr<AABB>
VoxelScenePart::geometryAABB(std::size_t index) const
{
  if (index >= centers.n_rows || index >= halfSizes.n_rows) {
    return nullptr;
  }
  glm::dvec3 const c = loadVec3(centers, index);
  glm::dvec3 const h = loadVec3(halfSizes, index);
  return std::make_shared<AABB>(c - h, c + h);
}

void
VoxelScenePart::geometryRotate(std::size_t index, Rotation& rotation)
{
  if (index >= centers.n_rows) {
    return;
  }
  storeVec3(centers, index, rotation.applyTo(loadVec3(centers, index)));
  if (index < normals.n_rows) {
    storeVec3(normals, index, rotation.applyTo(loadVec3(normals, index)));
  }
}

void
VoxelScenePart::geometryScale(std::size_t index, double factor)
{
  if (index >= centers.n_rows || index >= halfSizes.n_rows) {
    return;
  }
  storeVec3(centers, index, loadVec3(centers, index) * factor);
  storeVec3(halfSizes, index, loadVec3(halfSizes, index) * factor);
}

void
VoxelScenePart::geometryTranslate(std::size_t index, glm::dvec3 const& shift)
{
  if (index >= centers.n_rows) {
    return;
  }
  storeVec3(centers, index, loadVec3(centers, index) + shift);
}

void
VoxelScenePart::geometryUpdate(std::size_t)
{
  // No cache to update for the current bulk representation.
}

void
VoxelScenePart::deleteGeometries()
{
  clearGeometryStorage();
}

void
VoxelScenePart::clearGeometryStorage()
{
  centers.reset();
  halfSizes.reset();
  normals.reset();
  materialIndex.reset();
  materialTable.clear();
  centroid.reset();
  bound = nullptr;
}

std::size_t
VoxelScenePart::geometryVertexCount(std::size_t index) const
{
  return index < centers.n_rows ? 8 : 0;
}

std::size_t
VoxelScenePart::geometryDynamicVertexCount(std::size_t index) const
{
  return index < centers.n_rows ? 1 : 0;
}

glm::dvec3
VoxelScenePart::geometryDynamicVertexPosition(std::size_t geometryIndex,
                                              std::size_t vertexIndex) const
{
  if (geometryIndex >= centers.n_rows || vertexIndex != 0) {
    return glm::dvec3(0.0);
  }
  return loadVec3(centers, geometryIndex);
}

glm::dvec3
VoxelScenePart::geometryDynamicVertexNormal(std::size_t geometryIndex,
                                            std::size_t vertexIndex) const
{
  if (geometryIndex >= normals.n_rows || vertexIndex != 0) {
    return glm::dvec3(0.0);
  }
  return loadVec3(normals, geometryIndex);
}

void
VoxelScenePart::setGeometryDynamicVertexPosition(std::size_t geometryIndex,
                                                 std::size_t vertexIndex,
                                                 glm::dvec3 const& position)
{
  if (geometryIndex >= centers.n_rows || vertexIndex != 0) {
    return;
  }
  storeVec3(centers, geometryIndex, position);
}

void
VoxelScenePart::setGeometryDynamicVertexNormal(std::size_t geometryIndex,
                                               std::size_t vertexIndex,
                                               glm::dvec3 const& normal)
{
  if (geometryIndex >= normals.n_rows || vertexIndex != 0) {
    return;
  }
  storeVec3(normals, geometryIndex, normal);
}

double
VoxelScenePart::geometryGroundZOffset(std::size_t index) const
{
  if (index >= halfSizes.n_rows) {
    return 0.0;
  }
  return 2.0 *
         std::max(
           { halfSizes(index, 0), halfSizes(index, 1), halfSizes(index, 2) });
}

std::shared_ptr<Material>
VoxelScenePart::geometryMaterial(std::size_t index) const
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
VoxelScenePart::setGeometryMaterial(std::size_t index,
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
