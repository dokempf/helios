#include <AbstractGeometryFilter.h>
#include <DetailedVoxelScenePart.h>
#include <ScenePart.h>
#include <SwapOnRepeatHandler.h>
#include <TriangleScenePart.h>
#include <VoxelScenePart.h>
#include <XYZPointCloudFileLoader.h>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
SwapOnRepeatHandler::SwapOnRepeatHandler()
  : currentTimeToLive(1)
  , discardOnReplay(false)
  , holistic(false)
  , onSwapFirstPlay(false)
  , keepCRS(true)
  , baseline(nullptr)
  , null(false)
{
}

// ***   MAIN METHODS   *** //
// ************************ //
void
SwapOnRepeatHandler::swap(ScenePart& sp)
{
  --currentTimeToLive;
  if (currentTimeToLive < 1)
    doSwap(sp);
}

void
SwapOnRepeatHandler::prepare(ScenePart* sp)
{
  numTargetSwaps = (int)swapFilters.size();
  std::deque<int> ttls(timesToLive);
  numTargetReplays = 0;
  while (!ttls.empty()) {
    numTargetReplays += ttls.front();
    ttls.pop_front();
  }
  numCurrentSwaps = 0;
  if (sp == nullptr) {
    baseline = nullptr;
    return;
  }
  baseline = sp->clone(false);
  baseline->unbindGeometryOwners();
}

// ***  GETTERs and SETTERs  *** //
// ***************************** //
void
SwapOnRepeatHandler::pushSwapFilters(
  std::deque<AbstractGeometryFilter*> const& swapFilters)
{
  this->swapFilters.push_back(swapFilters);
}

void
SwapOnRepeatHandler::pushTimeToLive(int const timeToLive)
{
  this->timesToLive.push_back(timeToLive);
}

std::size_t
SwapOnRepeatHandler::baselineGeometryCount() const
{
  return baseline == nullptr ? 0 : baseline->geometryCount();
}

void
SwapOnRepeatHandler::setBaselineGeometryMaterial(
  std::size_t index,
  std::shared_ptr<Material> material)
{
  if (baseline == nullptr || index >= baseline->geometryCount()) {
    return;
  }
  baseline->setGeometryMaterial(index, material);
}

void
SwapOnRepeatHandler::releaseBaselineGeometries()
{
  if (baseline == nullptr) {
    return;
  }
  baseline->deleteGeometries();
}

void
SwapOnRepeatHandler::clearBaselineGeometryStorage()
{
  if (baseline != nullptr) {
    baseline->clearGeometryStorage();
  }
}

// ***  UTIL METHODS  *** //
// ********************** //
void
SwapOnRepeatHandler::doSwap(ScenePart& sp)
{
  // Get next queue of filters and update time to live
  std::deque<AbstractGeometryFilter*> filters = swapFilters.front();
  swapFilters.pop_front();
  currentTimeToLive = timesToLive.front();
  timesToLive.pop_front();

  // Apply filters
  bool firstIter = true;
  while (!filters.empty()) {
    // Get next filter
    AbstractGeometryFilter* filter = filters.front();
    filters.pop_front();
    // Run the filter
    ScenePart* genSP = filter->run();
    // Update the geometry if a new one has been loaded
    if (genSP != nullptr && genSP != std::addressof(sp)) {
      // Make holistic only if geometry is derived from a point cloud
      holistic = false;
      if (dynamic_cast<XYZPointCloudFileLoader*>(filter) != nullptr) {
        holistic = true;
      }
      // Free primitives memory from scene part
      sp.deleteGeometries();
      // The geometric swap itself
      doGeometricSwap(*genSP, sp);
      // Delete generated geometry (it will no longer be used)
      delete genSP;
    }
    // Otherwise
    else {
      // Reload the baseline geometry on the first iteration only
      if (firstIter && baseline != nullptr) {
        sp.deleteGeometries();
        std::shared_ptr<ScenePart> baselineClone = baseline->clone(false);
        doGeometricSwap(*baselineClone, sp);
      }
    }
    // Delete filter
    filter->primsOut = nullptr;
    delete filter;
    firstIter = false;
  }

  // Update current swaps count and activate first play flag
  ++numCurrentSwaps;
  onSwapFirstPlay = true;
}

void
SwapOnRepeatHandler::doGeometricSwap(ScenePart& src, ScenePart& dst)
{
  if (auto* dstDetailed = dynamic_cast<DetailedVoxelScenePart*>(&dst)) {
    if (auto* srcDetailed = dynamic_cast<DetailedVoxelScenePart*>(&src)) {
      *dstDetailed = *srcDetailed;
      return;
    }
  }
  if (auto* dstVoxel = dynamic_cast<VoxelScenePart*>(&dst)) {
    if (auto* srcVoxel = dynamic_cast<VoxelScenePart*>(&src)) {
      *dstVoxel = *srcVoxel;
      return;
    }
  }
  if (auto* dstTriangle = dynamic_cast<TriangleScenePart*>(&dst)) {
    if (auto* srcTriangle = dynamic_cast<TriangleScenePart*>(&src)) {
      *dstTriangle = *srcTriangle;
      return;
    }
  }

  // Legacy fallback for non-bulk scene parts.
  dst = src;
}
