#pragma once

#include <GeometryRef.h>
#include <PrintUtils.h>

#include <ostream>

/**
 * @brief Class representing a the intersection of a ray over a scene made
 *  of primitives
 * @see Scene
 */
class RaySceneIntersection
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Stable owner/index reference to intersected geometry.
   */
  GeometryRef geometryRef;
  /**
   * @brief Intersection point
   */
  glm::dvec3 point;
  /**
   * @brief Intersection incidence angle
   */
  double incidenceAngle = 0;
  /**
   * @brief The distance traversed by the ray until intersection
   */
  double hitDistance = 0;

  // ***  O P E R A T O R S  *** //
  // *************************** //
  friend std::ostream& operator<<(std::ostream& out, RaySceneIntersection& itst)
  {
    out << itst.geometryRef.index << "," << itst.point << ","
        << itst.incidenceAngle;
    return out;
  }
};
