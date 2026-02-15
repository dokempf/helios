#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <KDTreeRaycaster.h>
#include <SimpleKDTreeFactory.h>
#include <TriangleScenePart.h>

#include <memory>

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

TEST_CASE("KDTree: Build and raycast with geometry refs")
{
  std::shared_ptr<TriangleScenePart> part =
    std::make_shared<TriangleScenePart>(2);
  setTriangle(*part,
              0,
              glm::dvec3(-1.0, -1.0, 1.0),
              glm::dvec3(1.0, -1.0, 1.0),
              glm::dvec3(0.0, 1.0, 1.0));
  setTriangle(*part,
              1,
              glm::dvec3(-1.0, -1.0, 5.0),
              glm::dvec3(1.0, -1.0, 5.0),
              glm::dvec3(0.0, 1.0, 5.0));

  std::vector<GeometryRef> refs;
  refs.push_back({ part, 0 });
  refs.push_back({ part, 1 });

  SimpleKDTreeFactory factory;
  std::shared_ptr<LightKDTreeNode> root(
    factory.makeFromGeometryRefs(refs, true, false));
  REQUIRE(root != nullptr);

  KDTreeRaycaster raycaster(root);
  std::unique_ptr<RaySceneIntersection> hit(raycaster.search(
    glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.0, 0.0, 1.0), 0.0, 100.0, false));
  REQUIRE(hit != nullptr);
  REQUIRE(hit->geometryRef.isValid());
  REQUIRE(hit->geometryRef.index == 0);
  REQUIRE(hit->hitDistance == Catch::Approx(1.0));

  std::map<double, GeometryRef> allHits = raycaster.searchAll(
    glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.0, 0.0, 1.0), 0.0, 100.0, false);
  REQUIRE(allHits.size() == 2);
  REQUIRE(allHits.begin()->first == Catch::Approx(1.0));
}
