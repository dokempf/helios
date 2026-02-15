#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <AABB.h>
#include <Material.h>
#include <TriangleScenePart.h>

#include <cmath>

namespace {
inline void
setTriangle(TriangleScenePart& part,
            std::size_t index,
            glm::dvec3 const& a,
            glm::dvec3 const& b,
            glm::dvec3 const& c)
{
  part.vertices(index, 0) = a.x;
  part.vertices(index, 1) = a.y;
  part.vertices(index, 2) = a.z;
  part.vertices(index, 3) = b.x;
  part.vertices(index, 4) = b.y;
  part.vertices(index, 5) = b.z;
  part.vertices(index, 6) = c.x;
  part.vertices(index, 7) = c.y;
  part.vertices(index, 8) = c.z;
}
}

TEST_CASE("TriangleScenePart: Bulk geometry operations")
{
  TriangleScenePart part(1);
  setTriangle(part,
              0,
              glm::dvec3(0.0, 0.0, 0.0),
              glm::dvec3(1.0, 0.0, 0.0),
              glm::dvec3(0.0, 1.0, 0.0));

  REQUIRE(part.geometryCount() == 1);
  REQUIRE(part.geometryTypeOf() == ScenePart::GeometryType::TRIANGLE);

  glm::dvec3 const centroid = part.geometryCentroid(0);
  REQUIRE(centroid.x == Catch::Approx(1.0 / 3.0));
  REQUIRE(centroid.y == Catch::Approx(1.0 / 3.0));
  REQUIRE(centroid.z == Catch::Approx(0.0));

  std::shared_ptr<AABB> const box = part.geometryAABB(0);
  REQUIRE(box != nullptr);
  REQUIRE(box->getMin().x == Catch::Approx(0.0));
  REQUIRE(box->getMin().y == Catch::Approx(0.0));
  REQUIRE(box->getMin().z == Catch::Approx(0.0));
  REQUIRE(box->getMax().x == Catch::Approx(1.0));
  REQUIRE(box->getMax().y == Catch::Approx(1.0));
  REQUIRE(box->getMax().z == Catch::Approx(0.0));

  double const t = part.geometryRayIntersectionDistance(
    0, glm::dvec3(0.25, 0.25, 1.0), glm::dvec3(0.0, 0.0, -1.0));
  REQUIRE(t == Catch::Approx(1.0));

  std::vector<double> const times = part.geometryRayIntersection(
    0, glm::dvec3(0.25, 0.25, 1.0), glm::dvec3(0.0, 0.0, -1.0));
  REQUIRE(times.size() == 1);
  REQUIRE(times[0] == Catch::Approx(1.0));

  std::shared_ptr<AABB> const bound = part.computeBound();
  REQUIRE(bound != nullptr);
  REQUIRE(bound->getMax().x == Catch::Approx(1.0));

  std::shared_ptr<Material> material = std::make_shared<Material>();
  material->name = "triangle_mat";
  part.setGeometryMaterial(0, material);
  REQUIRE(part.geometryMaterial(0) == material);
}

TEST_CASE("TriangleScenePart: Transforms and dynamic vertices")
{
  TriangleScenePart part(1);
  setTriangle(part,
              0,
              glm::dvec3(0.0, 0.0, 0.0),
              glm::dvec3(1.0, 0.0, 0.0),
              glm::dvec3(0.0, 1.0, 0.0));

  Rotation rot(glm::dvec3(0.0, 0.0, 1.0), std::acos(-1.0) * 0.5);
  part.geometryRotate(0, rot);
  part.geometryScale(0, 2.0);
  part.geometryTranslate(0, glm::dvec3(1.0, 2.0, 3.0));

  glm::dvec3 const p0 = part.geometryDynamicVertexPosition(0, 0);
  glm::dvec3 const p1 = part.geometryDynamicVertexPosition(0, 1);
  glm::dvec3 const p2 = part.geometryDynamicVertexPosition(0, 2);

  REQUIRE(p0.x == Catch::Approx(1.0));
  REQUIRE(p0.y == Catch::Approx(2.0));
  REQUIRE(p0.z == Catch::Approx(3.0));
  REQUIRE(p1.x == Catch::Approx(1.0));
  REQUIRE(p1.y == Catch::Approx(4.0));
  REQUIRE(p1.z == Catch::Approx(3.0));
  REQUIRE(p2.x == Catch::Approx(-1.0));
  REQUIRE(p2.y == Catch::Approx(2.0));
  REQUIRE(p2.z == Catch::Approx(3.0));

  part.setGeometryDynamicVertexNormal(0, 0, glm::dvec3(0.0, 0.0, 1.0));
  glm::dvec3 const n0 = part.geometryDynamicVertexNormal(0, 0);
  REQUIRE(n0.z == Catch::Approx(1.0));
}
