#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <AABB.h>

TEST_CASE("AABB: Basic utility behavior")
{
  AABB box(glm::dvec3(-1.0, -2.0, -3.0), glm::dvec3(4.0, 5.0, 6.0));

  REQUIRE(box.getMin().x == Catch::Approx(-1.0));
  REQUIRE(box.getMin().y == Catch::Approx(-2.0));
  REQUIRE(box.getMin().z == Catch::Approx(-3.0));
  REQUIRE(box.getMax().x == Catch::Approx(4.0));
  REQUIRE(box.getMax().y == Catch::Approx(5.0));
  REQUIRE(box.getMax().z == Catch::Approx(6.0));

  glm::dvec3 const centroid = box.getCentroid();
  REQUIRE(centroid.x == Catch::Approx(1.5));
  REQUIRE(centroid.y == Catch::Approx(1.5));
  REQUIRE(centroid.z == Catch::Approx(1.5));

  std::vector<double> const t = box.getRayIntersection(
    glm::dvec3(0.0, 0.0, -10.0), glm::dvec3(0.0, 0.0, 1.0));
  REQUIRE(t.size() == 2);
  REQUIRE(t[0] == Catch::Approx(7.0));
  REQUIRE(t[1] == Catch::Approx(16.0));

  double const firstT = box.getRayIntersectionDistance(
    glm::dvec3(0.0, 0.0, -10.0), glm::dvec3(0.0, 0.0, 1.0));
  REQUIRE(firstT == Catch::Approx(7.0));
}

TEST_CASE("AABB: From vertices")
{
  std::vector<Vertex> vertices(3);
  vertices[0].pos = glm::dvec3(-2.0, 3.0, 1.0);
  vertices[1].pos = glm::dvec3(4.0, -1.0, 2.0);
  vertices[2].pos = glm::dvec3(0.0, 5.0, -4.0);

  std::shared_ptr<AABB> fromVertices = AABB::getForVertices(vertices);
  REQUIRE(fromVertices != nullptr);
  REQUIRE(fromVertices->getMin().x == Catch::Approx(-2.0));
  REQUIRE(fromVertices->getMin().y == Catch::Approx(-1.0));
  REQUIRE(fromVertices->getMin().z == Catch::Approx(-4.0));
  REQUIRE(fromVertices->getMax().x == Catch::Approx(4.0));
  REQUIRE(fromVertices->getMax().y == Catch::Approx(5.0));
  REQUIRE(fromVertices->getMax().z == Catch::Approx(2.0));
}
