#include <GroveKDTreeRaycaster.h>

#include <vector>

namespace {
std::vector<GeometryRef>
collectGeometryRefs(ScenePart& part)
{
  std::shared_ptr<ScenePart> owner(&part, [](ScenePart*) {});
  std::size_t const count = part.geometryCount();
  std::vector<GeometryRef> refs;
  refs.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    GeometryRef ref{ owner, i };
    if (ref.isValid()) {
      refs.push_back(ref);
    }
  }
  return refs;
}
} // namespace

// ***  GROVE DYNAMIC TREE METHODS  *** //
// ************************************ //
void
GroveKDTreeRaycaster::update(DynObject& dynObj)
{
  std::vector<GeometryRef> const dynObjRefs = collectGeometryRefs(dynObj);
  std::shared_ptr<std::vector<GeometryRef>> refs = sharedCopyRefs(dynObjRefs);

  cache_refs = refs;

  // Make new tree with an independent set of primitive references.
  // To prevent they from being updated by other threads when raycasting
  root = std::shared_ptr<LightKDTreeNode>(
    kdtf->makeFromGeometryRefs(*refs, true, false));
  // TODO Pending : Be careful how many threads kdtf is using because this
  // method is called during simulation, when other threads might be running
}

std::shared_ptr<GroveKDTreeRaycaster>
GroveKDTreeRaycaster::makeTemporalClone() const
{
  std::shared_ptr<GroveKDTreeRaycaster> gkdtr =
    std::make_shared<GroveKDTreeRaycaster>(root, nullptr, cache_refs);
  return gkdtr;
}

std::shared_ptr<std::vector<GeometryRef>>
GroveKDTreeRaycaster::sharedCopyRefs(std::vector<GeometryRef> const& src) const
{
  return std::make_shared<std::vector<GeometryRef>>(src);
}
