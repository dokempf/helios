#include <catch2/catch_test_macros.hpp>

#include <ScenePart.h>
#include <TriangleScenePart.h>

#include <vector>

TEST_CASE("ScenePart: Split subparts")
{
  // Build bulk triangle scene part.
  std::shared_ptr<TriangleScenePart> sp =
    std::make_shared<TriangleScenePart>(32);
  sp->vertices.zeros(32, 9);
  sp->normals.zeros(32, 9);
  sp->materialIndex.zeros(32);
  sp->materialTable.push_back(std::make_shared<Material>());
  for (size_t i = 0; i < 32; ++i) {
    sp->vertices(i, 0) = -1.0;
    sp->vertices(i, 1) = -1.0;
    sp->vertices(i, 2) = 0.0;
    sp->vertices(i, 3) = 0.0;
    sp->vertices(i, 4) = 0.0;
    sp->vertices(i, 5) = static_cast<double>(i) / 32.0;
    sp->vertices(i, 6) = 1.0;
    sp->vertices(i, 7) = 1.0;
    sp->vertices(i, 8) = 0.0;
    sp->materialIndex(i) = 0;
    if (i > 0 && (i % 4) == 0)
      sp->subpartLimit.push_back(i);
  }
  sp->subpartLimit.push_back(32);
  sp->mId = "0";

  // Split scene part
  std::vector<std::shared_ptr<ScenePart>> splitParts;
  REQUIRE(sp->splitSubparts(&splitParts));

  // Validate scene part splits.
  REQUIRE(sp->mId == "0");
  REQUIRE(sp->geometryCount() == 4);
  REQUIRE(splitParts.size() == 7);
  for (std::size_t i = 0; i < splitParts.size(); ++i) {
    REQUIRE(splitParts[i] != nullptr);
    REQUIRE(splitParts[i]->mId == std::to_string(i + 1));
    REQUIRE(splitParts[i]->geometryCount() == 4);
  }
}
