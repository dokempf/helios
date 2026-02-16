#include <adt/kdtree/KDTreeFactory.h>

class KDTreeFactoryWrap : public KDTreeFactory
{
public:
  using KDTreeFactory::KDTreeFactory; // Inherit constructors

  // Override pure virtual clone method
  KDTreeFactory* clone() const override
  {
    PYBIND11_OVERLOAD_PURE(KDTreeFactory*, // Return type
                           KDTreeFactory,  // Parent class
                           clone           // Function name
    );
  }

  KDTreeNodeRoot* makeFromGeometryRefsUnsafe(
    std::vector<GeometryRef>& geometryRefs,
    bool const computeStats = false,
    bool const reportStats = false) override
  {
    PYBIND11_OVERLOAD_PURE(KDTreeNodeRoot*,            // Return type
                           KDTreeFactory,              // Parent class
                           makeFromGeometryRefsUnsafe, // Method name
                           geometryRefs,               // Arguments
                           computeStats,
                           reportStats);
  }
};
