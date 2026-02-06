#include <catch2/catch_test_macros.hpp>

#include <FastSAHKDTreeFactory.h>
#include <GroveKDTreeRaycaster.h>
#include <KDGrove.h>
#include <ScenePart.h>

#include <memory>
#include <vector>

struct GroveTestFixture
{
  KDGrove kdg;
  std::vector<std::shared_ptr<GroveKDTreeRaycaster>> trees;
  std::vector<std::shared_ptr<ScenePart>> parts;

  GroveTestFixture()
  {
    buildPrimitives();
    buildTrees();
    buildGrove();
  }

  void buildPrimitives()
  {
    // First tree
    auto part1 = std::make_shared<ScenePart>();
    part1->primitiveType = ScenePart::PrimitiveType::TRIANGLE;
    appendTriangleBulk(part1->triangles,
                       Vertex(-1, -1, -1),
                       Vertex(1, -1, -1),
                       Vertex(0, 1, -1),
                       nullptr);
    appendTriangleBulk(part1->triangles,
                       Vertex(-1, -1, -1),
                       Vertex(1, -1, -1),
                       Vertex(0, 0, 1),
                       nullptr);
    appendTriangleBulk(part1->triangles,
                       Vertex(1, -1, -1),
                       Vertex(0, 0, 1),
                       Vertex(0, 1, -1),
                       nullptr);
    appendTriangleBulk(part1->triangles,
                       Vertex(0, 1, -1),
                       Vertex(0, 0, 1),
                       Vertex(-1, -1, -1),
                       nullptr);
    part1->buildPrimitiveViewsFromBulk();
    parts.push_back(part1);

    // Second tree
    auto part2 = std::make_shared<ScenePart>();
    part2->primitiveType = ScenePart::PrimitiveType::TRIANGLE;
    appendTriangleBulk(part2->triangles,
                       Vertex(0, 0, -1),
                       Vertex(2, 0, -1),
                       Vertex(1, 2, -1),
                       nullptr);
    appendTriangleBulk(part2->triangles,
                       Vertex(0, 0, -1),
                       Vertex(2, 0, -1),
                       Vertex(1, 1, 1),
                       nullptr);
    appendTriangleBulk(part2->triangles,
                       Vertex(2, 0, -1),
                       Vertex(1, 1, 1),
                       Vertex(1, 2, -1),
                       nullptr);
    appendTriangleBulk(part2->triangles,
                       Vertex(1, 2, -1),
                       Vertex(1, 1, 1),
                       Vertex(0, 0, -1),
                       nullptr);
    part2->buildPrimitiveViewsFromBulk();
    parts.push_back(part2);

    // Third tree
    auto part3 = std::make_shared<ScenePart>();
    part3->primitiveType = ScenePart::PrimitiveType::TRIANGLE;
    appendTriangleBulk(part3->triangles,
                       Vertex(4, 4, -1),
                       Vertex(6, 4, -1),
                       Vertex(5, 6, -1),
                       nullptr);
    appendTriangleBulk(part3->triangles,
                       Vertex(4, 4, -1),
                       Vertex(6, 4, -1),
                       Vertex(5, 5, 1),
                       nullptr);
    appendTriangleBulk(part3->triangles,
                       Vertex(6, 4, -1),
                       Vertex(5, 5, 1),
                       Vertex(5, 6, -1),
                       nullptr);
    appendTriangleBulk(part3->triangles,
                       Vertex(5, 6, -1),
                       Vertex(5, 5, 1),
                       Vertex(4, 4, -1),
                       nullptr);
    part3->buildPrimitiveViewsFromBulk();
    parts.push_back(part3);

    // Fourth tree
    auto part4 = std::make_shared<ScenePart>();
    part4->primitiveType = ScenePart::PrimitiveType::TRIANGLE;
    appendTriangleBulk(part4->triangles,
                       Vertex(9, 9, 0),
                       Vertex(11, 9, 0),
                       Vertex(10, 11, 0),
                       nullptr);
    appendTriangleBulk(part4->triangles,
                       Vertex(9, 9, 0),
                       Vertex(11, 9, 0),
                       Vertex(10, 10, -3),
                       nullptr);
    appendTriangleBulk(part4->triangles,
                       Vertex(11, 9, 0),
                       Vertex(10, 10, -3),
                       Vertex(10, 11, 0),
                       nullptr);
    appendTriangleBulk(part4->triangles,
                       Vertex(10, 11, 0),
                       Vertex(10, 10, -3),
                       Vertex(9, 9, 0),
                       nullptr);
    part4->buildPrimitiveViewsFromBulk();
    parts.push_back(part4);
  }

  void buildTrees()
  {
    FastSAHKDTreeFactory kdtf(32, 1, 1, 1);
    for (auto& part : parts) {
      std::vector<PrimitiveRef> refs;
      part->appendPrimitiveRefs(refs);
      std::shared_ptr<LightKDTreeNode> tree(
        kdtf.makeFromPrimitives(refs, true, false));
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
