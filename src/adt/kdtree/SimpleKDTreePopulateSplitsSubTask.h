#pragma once

#include <SharedSubTask.h>
#include <SharedTaskSequencer.h>
#include <assetloading/ScenePart.h>

#include <vector>

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Shared sub-task to populate splits when building a Simple KDTree.
 *  It is meant to be used at geometry-level parallelization context and only
 *  for right child nodes.
 * @see SharedSubTask
 * @see SimpleKDTreeFactory
 * @see SimpleKDTreeFactory::populateSplits
 * @see MultiThreadKDTreeFactory
 */
class SimpleKDTreePopulateSplitsSubTask : public SharedSubTask
{
protected:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Primitives of node being splitted
   */
  std::vector<PrimitiveRef> const& primitives;
  /**
   * @brief Index of axis defining the split
   */
  int const splitAxis;
  /**
   * @brief Position on given axis of the split point
   */
  double const splitPos;
  /**
   * @brief Where primitives of left split must be stored
   */
  std::vector<PrimitiveRef>& leftPrimitives;
  /**
   * @brief Where primitives of right split must be stored
   */
  std::vector<PrimitiveRef>& rightPrimitives;
  /**
   * @brief Index of primitive (inclusive) at which the sub-task must start
   *  to iterate
   */
  std::size_t const startPrimitive;
  /**
   * @brief Index of primitive (exclusive) at which the sub-task must end
   *  iterating
   */
  std::size_t const endPrimitive;
  /**
   * @brief Function to digest a primitive when populating splits
   * @see SimpleKDTreeFactory::onPopulateSplitsDigestPrimitive
   */
  std::function<void(PrimitiveRef p,
                     int const splitAxis,
                     double const splitPos,
                     std::vector<PrimitiveRef>& leftPrimitives,
                     std::vector<PrimitiveRef>& rightPrimitives)>
    onPopulateSplitsDigestPrimitive;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Main constructor for Simple KDTree populate splits sub-task
   */
  SimpleKDTreePopulateSplitsSubTask(
    std::shared_ptr<SharedTaskSequencer> ch,
    std::vector<PrimitiveRef> const& primitives,
    int const splitAxis,
    double const splitPos,
    std::vector<PrimitiveRef>& leftPrimitives,
    std::vector<PrimitiveRef>& rightPrimitives,
    size_t const startPrimitive,
    size_t const endPrimitive,
    std::function<void(PrimitiveRef p,
                       int const splitAxis,
                       double const splitPos,
                       std::vector<PrimitiveRef>& leftPrimitives,
                       std::vector<PrimitiveRef>& rightPrimitives)>
      onPopulateSplitsDigestPrimitive)
    : SharedSubTask(ch)
    , primitives(primitives)
    , splitAxis(splitAxis)
    , splitPos(splitPos)
    , leftPrimitives(leftPrimitives)
    , rightPrimitives(rightPrimitives)
    , startPrimitive(startPrimitive)
    , endPrimitive(endPrimitive)
    , onPopulateSplitsDigestPrimitive(onPopulateSplitsDigestPrimitive)
  {
  }
  ~SimpleKDTreePopulateSplitsSubTask() override {}

  // ***  RUNNABLE SHARED TASK  *** //
  // ****************************** //
  /**
   * @brief Implementation of the populate splits method
   * @see SharedSubTask::run
   * @see SimpleKDTreeFactory::populateSplits
   */
  void run() override
  {
    for (size_t pi = startPrimitive; pi < endPrimitive; ++pi) {
      onPopulateSplitsDigestPrimitive(
        primitives[pi], splitAxis, splitPos, leftPrimitives, rightPrimitives);
    }
  }
};
