#pragma once

#include <DynObject.h>
#include <KDTreeFactory.h>
#include <KDTreeRaycaster.h>
#include <LightKDTreeNode.h>
#include <RaycasterGroveTree.h>
#include <memory>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Grove KDTree Raycaster extends KDTreeRaycaster to make it compatible
 *  with groves by implementing the RaycasterGroveTree interface
 * @see RaycasterGroveTree
 * @see KDTreeRaycaster
 */
class GroveKDTreeRaycaster
  : public RaycasterGroveTree<DynObject>
  , public KDTreeRaycaster
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The KDTreeFactory to be used to rebuild the KDTree if necessary
   */
  std::shared_ptr<KDTreeFactory> kdtf;

  // CACHE ATTRIBUTES
  /**
   * @brief The cache of primitives defining the last state for the root node
   *  of the raycasting process
   */
  std::shared_ptr<ScenePart> cache_part;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default Grove KDTree ray caster constructor
   * @param root Root node of the KDTree
   */
  GroveKDTreeRaycaster(std::shared_ptr<LightKDTreeNode> root,
                       std::shared_ptr<KDTreeFactory> kdtf = nullptr,
                       std::shared_ptr<ScenePart> cache_part = nullptr)
    : KDTreeRaycaster(root)
    , kdtf(kdtf)
    , cache_part(cache_part)
  {
  }
  /**
   * @brief The destructor of Grove KDTree must destroy any cache-related
   *  resource that doesnt make sense after the time-of-live of the raycaster
   *  has finished
   */
  ~GroveKDTreeRaycaster() override = default;

  // ***  RAYCASTING METHODS  *** //
  // **************************** //
  /**
   * @see Raycaster::searchAll
   */
  std::map<double, Primitive*> searchAll(glm::dvec3 rayOrigin,
                                         glm::dvec3 rayDir,
                                         double tmin,
                                         double tmax,
                                         bool groundOnly) override
  {
    return KDTreeRaycaster::searchAll(
      rayOrigin, rayDir, tmin, tmax, groundOnly);
  }
  /**
   * @see Raycaster::search
   */
  RaySceneIntersection* search(glm::dvec3 rayOrigin,
                               glm::dvec3 rayDir,
                               double tmin,
                               double tmax,
                               bool groundOnly) override
  {
    return KDTreeRaycaster::search(rayOrigin, rayDir, tmin, tmax, groundOnly);
  }

  // ***  GROVE DYNAMIC TREE METHODS  *** //
  // ************************************ //
  /**
   * @brief Method to handle callbacks from updated dynamic objects
   * @param dynObj The updated dynamic object
   * @see RaycasterGroveTree::update
   */
  void update(DynObject& dynObj) override;

  /**
   * @brief Make a temporal clone of the GroveKDTreeRaycaster
   *
   * The temporal clone is meant to produce a temporal copy of the tree. If
   *  the original tree is updated, then the temporal copy should not be
   *  updated.
   *
   * @return Temporal clone of the GroveKDTreeRaycaster
   * @see KDGrove::makeTemporalClone
   */
  virtual std::shared_ptr<GroveKDTreeRaycaster> makeTemporalClone() const;

  /**
   * @brief Access the cached scene part snapshot.
   */
  inline std::shared_ptr<ScenePart> const& getCachePart() const
  {
    return cache_part;
  }
};
