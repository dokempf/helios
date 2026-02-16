#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <AbstractGeometryFilter.h>
#include <Material.h>
#include <SwapOnRepeatHandler.h>
#include <TriangleScenePart.h>

#include <deque>
#include <memory>

namespace {
inline std::shared_ptr<TriangleScenePart>
makeTrianglePartAtZ(double z)
{
  std::shared_ptr<TriangleScenePart> part =
    std::make_shared<TriangleScenePart>(1);
  part->vertices.zeros(1, 9);
  part->normals.zeros(1, 9);
  part->materialIndex.zeros(1);
  part->materialTable.push_back(std::make_shared<Material>());
  part->materialIndex(0) = 0;
  part->vertices(0, 0) = -1.0;
  part->vertices(0, 1) = -1.0;
  part->vertices(0, 2) = z;
  part->vertices(0, 3) = 1.0;
  part->vertices(0, 4) = -1.0;
  part->vertices(0, 5) = z;
  part->vertices(0, 6) = 0.0;
  part->vertices(0, 7) = 1.0;
  part->vertices(0, 8) = z;
  return part;
}

class StaticTriangleFilter : public AbstractGeometryFilter
{
public:
  explicit StaticTriangleFilter(std::shared_ptr<TriangleScenePart> replacement)
    : AbstractGeometryFilter(nullptr)
    , replacement(std::move(replacement))
  {
  }

  ScenePart* run() override
  {
    if (!replacement) {
      return nullptr;
    }
    return new TriangleScenePart(*replacement);
  }

private:
  std::shared_ptr<TriangleScenePart> replacement;
};
}

TEST_CASE("SwapOnRepeat: Bulk scene-part swapping and baseline lifecycle")
{
  TriangleScenePart current(*makeTrianglePartAtZ(0.0));
  SwapOnRepeatHandler handler;

  std::deque<AbstractGeometryFilter*> filters;
  filters.push_back(new StaticTriangleFilter(makeTrianglePartAtZ(10.0)));
  handler.pushSwapFilters(filters);
  handler.pushTimeToLive(1);
  handler.prepare(&current);

  REQUIRE(handler.baseline != nullptr);
  REQUIRE(handler.baselineGeometryCount() == 1);

  handler.swap(current);
  REQUIRE(current.geometryCentroid(0).z == Catch::Approx(10.0));

  std::shared_ptr<Material> baselineMat = std::make_shared<Material>();
  baselineMat->name = "baseline_mat";
  handler.setBaselineGeometryMaterial(0, baselineMat);
  REQUIRE(handler.baseline->geometryMaterial(0) == baselineMat);

  handler.releaseBaselineGeometries();
  REQUIRE(handler.baseline->geometryCount() == 0);

  handler.prepare(&current);
  REQUIRE(handler.baselineGeometryCount() == 1);
  handler.clearBaselineGeometryStorage();
  REQUIRE(handler.baseline->geometryCount() == 0);
}

TEST_CASE("SwapOnRepeat: Flags")
{
  SwapOnRepeatHandler handler;
  REQUIRE(handler.isKeepCRS());
  handler.setKeepCRS(false);
  REQUIRE_FALSE(handler.isKeepCRS());

  REQUIRE_FALSE(handler.isNull());
  handler.setNull(true);
  REQUIRE(handler.isNull());
}
