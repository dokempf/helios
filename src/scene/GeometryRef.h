#pragma once

#include <ScenePart.h>

#include <boost/serialization/access.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <cstddef>
#include <limits>
#include <memory>

class Material;

/**
 * @brief Stable reference to a geometry element through its owning ScenePart
 * and index.
 */
struct GeometryRef
{
private:
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive& ar, const unsigned int)
  {
    ar & part;
    ar & index;
  }

public:
  std::shared_ptr<ScenePart> part = nullptr;
  std::size_t index = std::numeric_limits<std::size_t>::max();

  static constexpr std::size_t invalidIndex()
  {
    return std::numeric_limits<std::size_t>::max();
  }

  inline bool isValid() const
  {
    return part != nullptr && index != invalidIndex() &&
           index < part->geometryCount();
  }

  inline ScenePart::GeometryType geometryType() const
  {
    return isValid() ? part->geometryTypeOf() : ScenePart::GeometryType::NONE;
  }

  inline glm::dvec3 centroid() const
  {
    return isValid() ? part->geometryCentroid(index) : glm::dvec3(0.0);
  }

  inline double rayIntersectionDistance(glm::dvec3 const& rayOrigin,
                                        glm::dvec3 const& rayDir) const
  {
    return isValid()
             ? part->geometryRayIntersectionDistance(index, rayOrigin, rayDir)
             : -1.0;
  }

  inline std::vector<double> rayIntersection(glm::dvec3 const& rayOrigin,
                                             glm::dvec3 const& rayDir) const
  {
    return isValid() ? part->geometryRayIntersection(index, rayOrigin, rayDir)
                     : std::vector<double>();
  }

  inline double incidenceAngle(glm::dvec3 const& rayOrigin,
                               glm::dvec3 const& rayDir,
                               glm::dvec3 const& intersectionPoint) const
  {
    return isValid() ? part->geometryIncidenceAngle(
                         index, rayOrigin, rayDir, intersectionPoint)
                     : 0.0;
  }

  inline std::shared_ptr<Material> material() const
  {
    return isValid() ? part->geometryMaterial(index) : nullptr;
  }

  inline double groundZOffset() const
  {
    return isValid() ? part->geometryGroundZOffset(index) : 0.0;
  }

  inline void reset()
  {
    part = nullptr;
    index = invalidIndex();
  }
};
