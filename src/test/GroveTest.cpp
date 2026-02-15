#include <catch2/catch_test_macros.hpp>

#include <FastSAHKDTreeFactory.h>
#include <GeometryRef.h>
#include <GroveKDTreeRaycaster.h>
#include <KDGrove.h>
#include <TriangleScenePart.h>

#include <memory>
#include <vector>

struct GroveTestFixture
{
  KDGrove kdg;
  std::vector<std::shared_ptr<GroveKDTreeRaycaster>> trees;
  std::vector<std::shared_ptr<TriangleScenePart>> parts;

  GroveTestFixture()
  {
    buildPrimitives();
    buildTrees();
    buildGrove();
  }

  ~GroveTestFixture() = default;

  void buildPrimitives()
  {
    auto makePart = []() -> std::shared_ptr<TriangleScenePart> {
      std::shared_ptr<TriangleScenePart> out =
        std::make_shared<TriangleScenePart>(4);
      out->vertices.zeros(4, 9);
      out->normals.zeros(4, 9);
      out->materialIndex.zeros(4);
      out->materialTable.push_back(std::make_shared<Material>());
      for (std::size_t i = 0; i < 4; ++i) {
        out->materialIndex(i) = 0;
      }
      return out;
    };
    auto setTriangle = [](TriangleScenePart& part,
                          std::size_t row,
                          glm::dvec3 const& a,
                          glm::dvec3 const& b,
                          glm::dvec3 const& c) {
      part.vertices(row, 0) = a.x;
      part.vertices(row, 1) = a.y;
      part.vertices(row, 2) = a.z;
      part.vertices(row, 3) = b.x;
      part.vertices(row, 4) = b.y;
      part.vertices(row, 5) = b.z;
      part.vertices(row, 6) = c.x;
      part.vertices(row, 7) = c.y;
      part.vertices(row, 8) = c.z;
    };

    // First tree
    std::shared_ptr<TriangleScenePart> tris1 = makePart();
    setTriangle(*tris1,
                0,
                glm::dvec3(-1, -1, -1),
                glm::dvec3(1, -1, -1),
                glm::dvec3(0, 1, -1));
    setTriangle(*tris1,
                1,
                glm::dvec3(-1, -1, -1),
                glm::dvec3(1, -1, -1),
                glm::dvec3(0, 0, 1));
    setTriangle(*tris1,
                2,
                glm::dvec3(1, -1, -1),
                glm::dvec3(0, 0, 1),
                glm::dvec3(0, 1, -1));
    setTriangle(*tris1,
                3,
                glm::dvec3(0, 1, -1),
                glm::dvec3(0, 0, 1),
                glm::dvec3(-1, -1, -1));
    parts.push_back(tris1);

    // Second tree
    std::shared_ptr<TriangleScenePart> tris2 = makePart();
    setTriangle(*tris2,
                0,
                glm::dvec3(0, 0, -1),
                glm::dvec3(2, 0, -1),
                glm::dvec3(1, 2, -1));
    setTriangle(*tris2,
                1,
                glm::dvec3(0, 0, -1),
                glm::dvec3(2, 0, -1),
                glm::dvec3(1, 1, 1));
    setTriangle(*tris2,
                2,
                glm::dvec3(2, 0, -1),
                glm::dvec3(1, 1, 1),
                glm::dvec3(1, 2, -1));
    setTriangle(*tris2,
                3,
                glm::dvec3(1, 2, -1),
                glm::dvec3(1, 1, 1),
                glm::dvec3(0, 0, -1));
    parts.push_back(tris2);

    // Third tree
    std::shared_ptr<TriangleScenePart> tris3 = makePart();
    setTriangle(*tris3,
                0,
                glm::dvec3(4, 4, -1),
                glm::dvec3(6, 4, -1),
                glm::dvec3(5, 6, -1));
    setTriangle(*tris3,
                1,
                glm::dvec3(4, 4, -1),
                glm::dvec3(6, 4, -1),
                glm::dvec3(5, 5, 1));
    setTriangle(*tris3,
                2,
                glm::dvec3(6, 4, -1),
                glm::dvec3(5, 5, 1),
                glm::dvec3(5, 6, -1));
    setTriangle(*tris3,
                3,
                glm::dvec3(5, 6, -1),
                glm::dvec3(5, 5, 1),
                glm::dvec3(4, 4, -1));
    parts.push_back(tris3);

    // Fourth tree
    std::shared_ptr<TriangleScenePart> tris4 = makePart();
    setTriangle(*tris4,
                0,
                glm::dvec3(9, 9, 0),
                glm::dvec3(11, 9, 0),
                glm::dvec3(10, 11, 0));
    setTriangle(*tris4,
                1,
                glm::dvec3(9, 9, 0),
                glm::dvec3(11, 9, 0),
                glm::dvec3(10, 10, -3));
    setTriangle(*tris4,
                2,
                glm::dvec3(11, 9, 0),
                glm::dvec3(10, 10, -3),
                glm::dvec3(10, 11, 0));
    setTriangle(*tris4,
                3,
                glm::dvec3(10, 11, 0),
                glm::dvec3(10, 10, -3),
                glm::dvec3(9, 9, 0));
    parts.push_back(tris4);
  }

  void buildTrees()
  {
    FastSAHKDTreeFactory kdtf(32, 1, 1, 1);
    for (std::shared_ptr<TriangleScenePart> const& tris : parts) {
      std::vector<GeometryRef> refs;
      refs.reserve(tris->geometryCount());
      for (std::size_t i = 0; i < tris->geometryCount(); ++i) {
        refs.push_back({ tris, i });
      }
      std::shared_ptr<LightKDTreeNode> tree(
        kdtf.makeFromGeometryRefs(refs, true, false));
      trees.push_back(std::make_shared<GroveKDTreeRaycaster>(tree));
    }
  }

  void buildGrove()
  {
    for (auto& tree : trees) {
      kdg.addTree(tree);
    }
  }
};

TEST_CASE("GroveTest: Loop mechanics")
{
  GroveTestFixture fixture;
  auto& kdg = fixture.kdg;
  auto& trees = fixture.trees;

  size_t i, j, iMax, m;

  // For loop test
  m = kdg.getNumTrees();
  for (j = 0; j < 3; ++j) {
    for (i = 0, iMax = 0; i < m; ++i) {
      REQUIRE(trees[i] == kdg[i]);
      if (i > iMax)
        iMax = i;
    }
  }
  REQUIRE(iMax == m - 1);
  REQUIRE(m == trees.size());

  // While loop test
  for (j = 0; j < 3; ++j) {
    i = 0;
    kdg.toZeroTree();
    while (kdg.hasNextTree()) {
      REQUIRE(trees[i] == kdg.nextTreeShared());
      ++i;
    }
    REQUIRE(i == m);
  }

  // For-each loop test
  for (j = 0; j < 3; ++j) {
    i = 0;
    for (auto gkdt : kdg) {
      REQUIRE(trees[i] == gkdt);
      ++i;
    }
    REQUIRE(i == m);
  }
}

TEST_CASE("GroveTest: Observer pattern mechanics")
{
  GroveTestFixture fixture;
  auto& kdg = fixture.kdg;

  auto tree = kdg.getTreeShared(0);
  DynMovingObject dmo1("dmo1");
  DynMovingObject dmo2("dmo2");

  // Add dmo1, dmo2 and validate
  kdg.addSubject(&dmo1, tree);
  kdg.addSubject(&dmo2, tree);
  REQUIRE(static_cast<DynMovingObject*>(kdg.getSubjects()[4])->getId() ==
          "dmo1");
  REQUIRE(static_cast<DynMovingObject*>(kdg.getSubjects()[5])->getId() ==
          "dmo2");

  // Remove dmo1 and validate
  kdg.removeSubject(&dmo1);
  REQUIRE(static_cast<DynMovingObject*>(kdg.getSubjects()[4])->getId() ==
          "dmo2");

  // Remove dmo2 and validate
  kdg.removeSubject(&dmo2);
  REQUIRE(kdg.getSubjects().size() <= 4);
}
