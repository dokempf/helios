#include <catch2/catch_test_macros.hpp>

#include <ScenePart.h>

#include <cstdlib>
#include <memory>
#include <vector>
TEST_CASE("ScenePart: Split subparts")
{
  // Build scene part
  auto material = std::make_shared<Material>();
  std::shared_ptr<ScenePart> sp = std::make_shared<ScenePart>();
  sp->primitiveType = ScenePart::PrimitiveType::TRIANGLE;
  for (size_t i = 0; i < 32; ++i) {
    if (i > 0 && (i % 4) == 0)
      sp->subpartLimit.push_back(i);
  }
  sp->subpartLimit.push_back(32);
  sp->mId = "0";
  sp->triangles.vertices.reserve(32 * 3);
  for (size_t i = 0; i < 32; ++i) {
    Vertex v0, v1, v2;
    v0.pos = glm::dvec3(-1.0, -1.0, 0.0);
    v1.pos = glm::dvec3(0.0, 0.0, ((double)i) / 32.0);
    v2.pos = glm::dvec3(1.0, 1.0, 0.0);
    appendTriangleBulk(sp->triangles, v0, v1, v2, material);
  }

  // Split scene part
  std::vector<std::shared_ptr<ScenePart>> parts = { sp };
  std::vector<std::shared_ptr<ScenePart>> newParts = sp->splitSubparts();
  parts.insert(parts.end(), newParts.begin(), newParts.end());

  // Validate scene part splits
  REQUIRE(parts.size() == 8);
  for (size_t i = 0; i < parts.size(); ++i) {
    REQUIRE(parts[i]->triangles.size() == 4);
    REQUIRE(std::atoi(parts[i]->mId.c_str()) == (int)i);
  }
}
