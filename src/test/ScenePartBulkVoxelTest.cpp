#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <AABB.h>
#include <Material.h>
#include <VoxelScenePart.h>

TEST_CASE("VoxelScenePart: Bulk geometry operations")
{
  VoxelScenePart part(1);
  part.centers(0, 0) = 1.0;
  part.centers(0, 1) = 2.0;
  part.centers(0, 2) = 3.0;
  part.halfSizes(0, 0) = 0.5;
  part.halfSizes(0, 1) = 0.5;
  part.halfSizes(0, 2) = 0.5;
  part.normals(0, 0) = 0.0;
  part.normals(0, 1) = 0.0;
  part.normals(0, 2) = 1.0;

  REQUIRE(part.geometryCount() == 1);
  REQUIRE(part.geometryTypeOf() == ScenePart::GeometryType::VOXEL);

  std::shared_ptr<AABB> const box = part.geometryAABB(0);
  REQUIRE(box != nullptr);
  REQUIRE(box->getMin().x == Catch::Approx(0.5));
  REQUIRE(box->getMin().y == Catch::Approx(1.5));
  REQUIRE(box->getMin().z == Catch::Approx(2.5));
  REQUIRE(box->getMax().x == Catch::Approx(1.5));
  REQUIRE(box->getMax().y == Catch::Approx(2.5));
  REQUIRE(box->getMax().z == Catch::Approx(3.5));

  double const t = part.geometryRayIntersectionDistance(
    0, glm::dvec3(1.0, 2.0, 0.0), glm::dvec3(0.0, 0.0, 1.0));
  REQUIRE(t == Catch::Approx(2.5));

  REQUIRE(part.geometryGroundZOffset(0) == Catch::Approx(1.0));

  std::shared_ptr<Material> material = std::make_shared<Material>();
  material->name = "voxel_mat";
  part.setGeometryMaterial(0, material);
  REQUIRE(part.geometryMaterial(0) == material);
}

TEST_CASE("VoxelScenePart: Transform and dynamic updates")
{
  VoxelScenePart part(1);
  part.centers(0, 0) = 1.0;
  part.centers(0, 1) = 2.0;
  part.centers(0, 2) = 3.0;
  part.halfSizes(0, 0) = 0.5;
  part.halfSizes(0, 1) = 0.5;
  part.halfSizes(0, 2) = 0.5;
  part.normals(0, 0) = 1.0;
  part.normals(0, 1) = 0.0;
  part.normals(0, 2) = 0.0;

  Rotation rot(glm::dvec3(0.0, 0.0, 1.0), std::acos(-1.0) * 0.5);
  part.geometryRotate(0, rot);
  part.geometryScale(0, 2.0);
  part.geometryTranslate(0, glm::dvec3(1.0, 0.0, 0.0));

  glm::dvec3 const center = part.geometryDynamicVertexPosition(0, 0);
  REQUIRE(center.x == Catch::Approx(-3.0));
  REQUIRE(center.y == Catch::Approx(2.0));
  REQUIRE(center.z == Catch::Approx(6.0));

  glm::dvec3 const normal = part.geometryDynamicVertexNormal(0, 0);
  REQUIRE(normal.x == Catch::Approx(0.0).margin(1e-12));
  REQUIRE(normal.y == Catch::Approx(1.0).margin(1e-12));
  REQUIRE(normal.z == Catch::Approx(0.0).margin(1e-12));

  part.setGeometryDynamicVertexPosition(0, 0, glm::dvec3(10.0, 11.0, 12.0));
  REQUIRE(part.geometryDynamicVertexPosition(0, 0).x == Catch::Approx(10.0));

  part.setGeometryDynamicVertexNormal(0, 0, glm::dvec3(0.0, 0.0, -1.0));
  REQUIRE(part.geometryDynamicVertexNormal(0, 0).z == Catch::Approx(-1.0));
}
