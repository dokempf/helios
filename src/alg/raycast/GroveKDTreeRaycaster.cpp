#include <GroveKDTreeRaycaster.h>
#include <scene/primitives/PrimitiveAccessor.h>

// ***  GROVE DYNAMIC TREE METHODS  *** //
// ************************************ //
void
GroveKDTreeRaycaster::update(DynObject& dynObj)
{
  // Snapshot current dynamic object geometry into a scene part
  std::shared_ptr<ScenePart> snapshot = std::make_shared<ScenePart>(dynObj);
  cache_part = snapshot;

  // Make new tree with its independent set of primitives
  // To prevent they from being updated by other threads when raycasting
  std::vector<PrimitiveRef> primRefs;
  snapshot->appendPrimitiveRefs(primRefs);
  root = std::shared_ptr<LightKDTreeNode>(
    kdtf->makeFromPrimitives(primRefs, true, false));
  // TODO Pending : Be careful how many threads kdtf is using because this
  // method is called during simulation, when other threads might be running
}

std::shared_ptr<GroveKDTreeRaycaster>
GroveKDTreeRaycaster::makeTemporalClone() const
{
  std::shared_ptr<GroveKDTreeRaycaster> gkdtr =
    std::make_shared<GroveKDTreeRaycaster>(root, nullptr, cache_part);
  return gkdtr;
}
