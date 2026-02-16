#pragma once

#include <GeometryRef.h>

/**
 * @brief KDTree primitive comparator
 */
struct KDTreeGeometryComparator
{
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief Index of axis considered for the comparison
   */
  int axis;

public:
  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for KDTree primitive comparator
   * @param axis
   */
  explicit KDTreeGeometryComparator(int axis) { this->axis = axis; }

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @brief KDTree primitive comparator functor
   * @param a First primitive for the comparison
   * @param b Second primitive for the comparison
   * @return True of coordinate of primitive a at corresponding axis is
   *  greater than coordinate of primitive b at the same axis, false
   *  otherwise
   */
  bool operator()(GeometryRef const& a, GeometryRef const& b) const
  {
    auto const ax = a.centroid()[axis];
    auto const bx = b.centroid()[axis];
    if (ax > bx)
      return true;
    return false;
  }
};
