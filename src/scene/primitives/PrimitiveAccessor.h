#pragma once

#include <assetloading/ScenePart.h>

#include <AABB.h>
#include <IntersectionHandlingResult.h>
#include <NoiseSource.h>

/**
 * @brief Helper utilities to access bulk primitive data by index.
 */
class PrimitiveAccessor
{
public:
  static inline size_t getNumVertices(PrimitiveRef const& ref)
  {
    if (ref.type == ScenePart::PrimitiveType::TRIANGLE)
      return 3;
    if (ref.type == ScenePart::PrimitiveType::VOXEL)
      return 1;
    return 0;
  }

  static Vertex* getVertices(PrimitiveRef const& ref);
  static glm::dvec3 getCentroid(PrimitiveRef const& ref);
  static AABB getAABB(PrimitiveRef const& ref);
  static double getRayIntersectionDistance(PrimitiveRef const& ref,
                                           glm::dvec3 const& rayOrigin,
                                           glm::dvec3 const& rayDir);
  static std::vector<double> getRayIntersection(PrimitiveRef const& ref,
                                                glm::dvec3 const& rayOrigin,
                                                glm::dvec3 const& rayDir);
  static double getGroundZOffset(PrimitiveRef const& ref);
  static std::shared_ptr<Material> getMaterial(PrimitiveRef const& ref);
  static bool isGround(PrimitiveRef const& ref);

  static double getIncidenceAngle_rad(PrimitiveRef const& ref,
                                      glm::dvec3 const& rayOrigin,
                                      glm::dvec3 const& rayDir,
                                      glm::dvec3 const& intersectionPoint);
  static bool canHandleIntersections(PrimitiveRef const& ref);
  static IntersectionHandlingResult onRayIntersection(
    PrimitiveRef const& ref,
    NoiseSource<double>& uniformNoiseSource,
    glm::dvec3& rayDirection,
    glm::dvec3 const& insideIntersectionPoint,
    glm::dvec3 const& outsideIntersectionPoint,
    double rayIntensity);
  static void onFinishLoading(PrimitiveRef const& ref,
                              NoiseSource<double>& uniformNoiseSource);
  static bool canComputeSigmaWithLadLut(PrimitiveRef const& ref);
  static double computeSigmaWithLadLut(PrimitiveRef const& ref,
                                       glm::dvec3 const& direction);
};
