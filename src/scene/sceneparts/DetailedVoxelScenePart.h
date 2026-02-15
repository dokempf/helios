#pragma once

#include <VoxelScenePart.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>

#include <cstdint>
#include <vector>

/**
 * @brief Bulk detailed-voxel-backed ScenePart.
 */
class DetailedVoxelScenePart : public VoxelScenePart
{
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive& ar, const unsigned int)
  {
    ar& boost::serialization::base_object<VoxelScenePart>(*this);

    std::size_t intRows = intData.n_rows;
    std::size_t intCols = intData.n_cols;
    ar & intRows;
    ar & intCols;
    std::vector<int> intFlat;
    if constexpr (Archive::is_saving::value) {
      intFlat.resize(intRows * intCols, 0);
      for (std::size_t r = 0; r < intRows; ++r) {
        for (std::size_t c = 0; c < intCols; ++c) {
          intFlat[r * intCols + c] = intData(r, c);
        }
      }
      ar & intFlat;
    } else {
      ar & intFlat;
      intData.set_size(intRows, intCols);
      for (std::size_t r = 0; r < intRows; ++r) {
        for (std::size_t c = 0; c < intCols; ++c) {
          intData(r, c) = intFlat[r * intCols + c];
        }
      }
    }

    ar & doubleData;

    std::size_t idCount = identifiers.n_elem;
    ar & idCount;
    std::vector<std::uint64_t> ids;
    if constexpr (Archive::is_saving::value) {
      ids.resize(idCount, 0);
      for (std::size_t i = 0; i < idCount; ++i) {
        ids[i] = identifiers(i);
      }
      ar & ids;
    } else {
      ar & ids;
      identifiers.set_size(idCount);
      for (std::size_t i = 0; i < idCount; ++i) {
        identifiers(i) = ids[i];
      }
    }

    ar & maxPad;
  }

public:
  arma::Mat<int> intData;
  arma::mat doubleData;
  arma::uvec identifiers;
  double maxPad = 0.0;

  DetailedVoxelScenePart();
  explicit DetailedVoxelScenePart(std::size_t voxelCount);
  ~DetailedVoxelScenePart() override = default;
  std::shared_ptr<ScenePart> clone(bool shallowGeometry = false) const override;

  GeometryType geometryTypeOf() const override;
  bool geometryCanComputeSigmaWithLadLut(std::size_t index) const override;
  double geometryComputeSigmaWithLadLut(
    std::size_t index,
    glm::dvec3 const& direction) const override;
  bool geometryCanHandleIntersections(std::size_t index) const override;
  IntersectionHandlingResult geometryOnRayIntersection(
    std::size_t index,
    NoiseSource<double>& uniformNoiseSource,
    glm::dvec3& rayDirection,
    glm::dvec3 const& insideIntersectionPoint,
    glm::dvec3 const& outsideIntersectionPoint,
    double rayIntensity) const override;
  void geometryOnFinishLoading(
    std::size_t index,
    NoiseSource<double>& uniformNoiseSource) override;
};
