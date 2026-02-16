#pragma once

#include <ScenePart.h>

#include <Material.h>
#include <armadillo>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>

#include <cstdint>
#include <vector>

/**
 * @brief Bulk voxel-backed ScenePart.
 */
class VoxelScenePart : public ScenePart
{
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive& ar, const unsigned int)
  {
    ar& boost::serialization::base_object<ScenePart>(*this);
    ar & centers;
    ar & halfSizes;
    ar & normals;
    ar & materialTable;

    std::size_t materialCount = materialIndex.n_elem;
    ar & materialCount;
    std::vector<std::uint64_t> materialIdx;
    if constexpr (Archive::is_saving::value) {
      materialIdx.resize(materialCount, 0);
      for (std::size_t i = 0; i < materialCount; ++i) {
        materialIdx[i] = materialIndex(i);
      }
      ar & materialIdx;
    } else {
      ar & materialIdx;
      materialIndex.set_size(materialCount);
      for (std::size_t i = 0; i < materialCount; ++i) {
        materialIndex(i) = materialIdx[i];
      }
    }
  }

public:
  arma::mat centers;   // n x 3
  arma::mat halfSizes; // n x 3
  arma::mat normals;   // optional n x 3
  std::vector<std::shared_ptr<Material>> materialTable;
  arma::uvec materialIndex;

  VoxelScenePart();
  explicit VoxelScenePart(std::size_t voxelCount);
  ~VoxelScenePart() override = default;
  std::shared_ptr<ScenePart> clone(bool shallowGeometry = false) const override;

  std::size_t geometryCount() const override;
  GeometryType geometryTypeOf() const override;
  std::shared_ptr<AABB> computeBound() override;
  glm::dvec3 geometryCentroid(std::size_t index) const override;
  double geometryRayIntersectionDistance(
    std::size_t index,
    glm::dvec3 const& rayOrigin,
    glm::dvec3 const& rayDir) const override;
  std::vector<double> geometryRayIntersection(
    std::size_t index,
    glm::dvec3 const& rayOrigin,
    glm::dvec3 const& rayDir) const override;
  double geometryIncidenceAngle(
    std::size_t index,
    glm::dvec3 const& rayOrigin,
    glm::dvec3 const& rayDir,
    glm::dvec3 const& intersectionPoint) const override;
  std::shared_ptr<AABB> geometryAABB(std::size_t index) const override;
  void geometryRotate(std::size_t index, Rotation& rotation) override;
  void geometryScale(std::size_t index, double factor) override;
  void geometryTranslate(std::size_t index, glm::dvec3 const& shift) override;
  void geometryUpdate(std::size_t index) override;
  void deleteGeometries() override;
  void clearGeometryStorage() override;
  std::size_t geometryVertexCount(std::size_t index) const override;
  std::size_t geometryDynamicVertexCount(std::size_t index) const override;
  glm::dvec3 geometryDynamicVertexPosition(
    std::size_t geometryIndex,
    std::size_t vertexIndex) const override;
  glm::dvec3 geometryDynamicVertexNormal(
    std::size_t geometryIndex,
    std::size_t vertexIndex) const override;
  void setGeometryDynamicVertexPosition(std::size_t geometryIndex,
                                        std::size_t vertexIndex,
                                        glm::dvec3 const& position) override;
  void setGeometryDynamicVertexNormal(std::size_t geometryIndex,
                                      std::size_t vertexIndex,
                                      glm::dvec3 const& normal) override;
  double geometryGroundZOffset(std::size_t index) const override;
  std::shared_ptr<Material> geometryMaterial(std::size_t index) const override;
  void setGeometryMaterial(std::size_t index,
                           std::shared_ptr<Material> material) override;
};
