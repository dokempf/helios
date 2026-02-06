#pragma once

#include <cmath>
#include <memory>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>

#include <AABB.h>
#include <IntersectionHandlingResult.h>
#include <NoiseSource.h>
#include <Primitive.h>
#include <ScenePart.h>

class TriangleView : public Primitive
{
public:
  ScenePart* owner = nullptr;
  PrimitiveIndex index = 0;

  TriangleView() = default;
  TriangleView(ScenePart* owner_, PrimitiveIndex index_)
    : owner(owner_)
    , index(index_)
  {
  }

  Primitive* clone() override;
  AABB* getAABB() override;
  glm::dvec3 getCentroid() override;
  double getIncidenceAngle_rad(const glm::dvec3& rayOrigin,
                               const glm::dvec3& rayDir,
                               const glm::dvec3& intersectionPoint) override;
  std::vector<double> getRayIntersection(const glm::dvec3& rayOrigin,
                                         const glm::dvec3& rayDir) override;
  double getRayIntersectionDistance(const glm::dvec3& rayOrigin,
                                    const glm::dvec3& rayDir) override;
  size_t getNumVertices() override { return 3; }
  Vertex* getVertices() override;
  void update() override;

  glm::dvec3 getFaceNormal() const;
  void setAllVertexColors(Color4f color);
  void setAllVertexNormalsFromFace();
  double calcArea2D();
  double calcArea3D();

private:
  mutable AABB aabb_cache;
  void updateAabbCache() const;
};

class VoxelView : public Primitive
{
public:
  ScenePart* owner = nullptr;
  PrimitiveIndex index = 0;

  VoxelView() = default;
  VoxelView(ScenePart* owner_, PrimitiveIndex index_)
    : owner(owner_)
    , index(index_)
  {
  }

  Primitive* clone() override;
  AABB* getAABB() override;
  glm::dvec3 getCentroid() override;
  double getIncidenceAngle_rad(const glm::dvec3& rayOrigin,
                               const glm::dvec3& rayDir,
                               const glm::dvec3& intersectionPoint) override;
  std::vector<double> getRayIntersection(const glm::dvec3& rayOrigin,
                                         const glm::dvec3& rayDir) override;
  double getRayIntersectionDistance(const glm::dvec3& rayOrigin,
                                    const glm::dvec3& rayDir) override;
  size_t getNumVertices() override { return 1; }
  Vertex* getVertices() override;
  size_t getNumFullVertices() override { return 2; }
  Vertex* getFullVertices() override;
  double getGroundZOffset() override;
  void update() override;

protected:
  bool hasNormal() const;
  double getIncidenceAngleClosestFace_rad(const glm::dvec3& rayOrigin,
                                          const glm::dvec3& rayDir,
                                          const glm::dvec3& intersectionPoint);

private:
  mutable AABB aabb_cache;
  void updateAabbCache() const;
};

class DetailedVoxelView : public VoxelView
{
public:
  DetailedVoxelView() = default;
  DetailedVoxelView(ScenePart* owner_, PrimitiveIndex index_)
    : VoxelView(owner_, index_)
  {
  }

  Primitive* clone() override;
  bool canHandleIntersections() override { return true; }
  IntersectionHandlingResult onRayIntersection(
    NoiseSource<double>& uniformNoiseSource,
    glm::dvec3& rayDirection,
    glm::dvec3 const& insideIntersectionPoint,
    glm::dvec3 const& outsideIntersectionPoint,
    double rayIntensity) override;
  void onFinishLoading(NoiseSource<double>& uniformNoiseSource) override;
  bool canComputeSigmaWithLadLut() override;
  double computeSigmaWithLadLut(glm::dvec3 const& direction) override;

private:
  IntersectionHandlingResult onRayIntersectionTransmittive(
    NoiseSource<double>& uniformNoiseSource,
    glm::dvec3& rayDirection,
    glm::dvec3 const& insideIntersectionPoint,
    glm::dvec3 const& outsideIntersectionPoint,
    double rayIntensity);
  IntersectionHandlingResult onRayIntersectionScaled(
    NoiseSource<double>& uniformNoiseSource,
    glm::dvec3& rayDirection,
    glm::dvec3 const& insideIntersectionPoint,
    glm::dvec3 const& outsideIntersectionPoint,
    double rayIntensity,
    double scaleFactor);
  IntersectionHandlingResult onRayIntersectionFixed(
    NoiseSource<double>& uniformNoiseSource,
    glm::dvec3& rayDirection,
    glm::dvec3 const& insideIntersectionPoint,
    glm::dvec3 const& outsideIntersectionPoint,
    double rayIntensity,
    double fixedSize);

  double getDoubleValueByIndex(size_t index) const;
};

struct ScenePartPrimitiveCache
{
  std::vector<TriangleView> triangleViews;
  std::vector<VoxelView> voxelViews;
  std::vector<DetailedVoxelView> detailedVoxelViews;
};

inline bool
isPrimitiveView(Primitive* p)
{
  return dynamic_cast<TriangleView*>(p) != nullptr ||
         dynamic_cast<VoxelView*>(p) != nullptr ||
         dynamic_cast<DetailedVoxelView*>(p) != nullptr;
}
