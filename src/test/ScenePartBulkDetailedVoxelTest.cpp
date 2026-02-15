#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <DetailedVoxelScenePart.h>
#include <IntersectionHandlingResult.h>
#include <UniformNoiseSource.h>

TEST_CASE("DetailedVoxelScenePart: Intersection handling modes")
{
  DetailedVoxelScenePart part(1);
  part.centers(0, 0) = 0.0;
  part.centers(0, 1) = 0.0;
  part.centers(0, 2) = 0.0;
  part.halfSizes(0, 0) = 1.0;
  part.halfSizes(0, 1) = 1.0;
  part.halfSizes(0, 2) = 1.0;
  part.doubleData.zeros(1, 9);
  part.intData.zeros(1, 1);
  part.identifiers.zeros(1);

  UniformNoiseSource<double> uns("1", 0.0, 1.0);
  glm::dvec3 rayDir(1.0, 0.0, 0.0);
  glm::dvec3 const inside(0.0, 0.0, 0.0);
  glm::dvec3 const outside(1.0, 0.0, 0.0);

  part.onRayIntersectionMode = "FIXED";
  part.doubleData(0, 8) = 1.0;
  IntersectionHandlingResult fixedKeep =
    part.geometryOnRayIntersection(0, uns, rayDir, inside, outside, 1.0);
  REQUIRE(fixedKeep.canRayContinue());

  part.doubleData(0, 8) = 0.0;
  IntersectionHandlingResult fixedStop =
    part.geometryOnRayIntersection(0, uns, rayDir, inside, outside, 1.0);
  REQUIRE_FALSE(fixedStop.canRayContinue());

  part.onRayIntersectionMode = "TRANSMITTIVE";
  part.doubleData(0, 0) = 1000000.0;
  IntersectionHandlingResult transmittive =
    part.geometryOnRayIntersection(0, uns, rayDir, inside, outside, 1.0);
  REQUIRE_FALSE(transmittive.canRayContinue());
}

TEST_CASE("DetailedVoxelScenePart: Scaled mode loading update")
{
  DetailedVoxelScenePart part(1);
  part.centers(0, 0) = 0.0;
  part.centers(0, 1) = 0.0;
  part.centers(0, 2) = 0.0;
  part.halfSizes(0, 0) = 2.0;
  part.halfSizes(0, 1) = 2.0;
  part.halfSizes(0, 2) = 2.0;
  part.doubleData.zeros(1, 1);
  part.doubleData(0, 0) = 1.0;
  part.maxPad = 4.0;
  part.onRayIntersectionMode = "SCALED";
  part.onRayIntersectionArgument = 1.0;

  UniformNoiseSource<double> uns("2", 0.0, 1.0);
  part.geometryOnFinishLoading(0, uns);

  REQUIRE(part.geometryTypeOf() == ScenePart::GeometryType::DETAILED_VOXEL);
  REQUIRE(part.halfSizes(0, 0) == Catch::Approx(0.5));
  REQUIRE(part.halfSizes(0, 1) == Catch::Approx(0.5));
  REQUIRE(part.halfSizes(0, 2) == Catch::Approx(0.5));
}
