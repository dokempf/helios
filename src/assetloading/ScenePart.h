#pragma once

#include <assetloading/SwapOnRepeatHandler.h>
class AABB;
class WavefrontObj;
class Material;
class IntersectionHandlingResult;
template<typename T>
class NoiseSource;

#include <LadLut.h>
#include <Vertex.h>
#include <maths/Rotation.h>

#include <armadillo>
#include <boost/serialization/version.hpp>
#include <iostream>
#include <memory>
#include <ogr_geometry.h>
#include <stdexcept>
#include <vector>

/**
 * @brief Class representing a scene part
 */
class ScenePart
{
  // ***  SERIALIZATION  *** //
  // *********************** //
  friend class boost::serialization::access;
  /**
   * @brief Serialize a ScenePart to a stream of bytes
   * @tparam Archive Type of rendering
   * @param ar Specific rendering for the stream of bytes
   * @param version Version number for the ScenePart
   */
  template<class Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    if (version < 2) {
      throw std::runtime_error(
        "Unsupported ScenePart serialization version. "
        "This build requires ScenePart archive version >= 2.");
    }
    ar & centroid;
    ar & bound;
    ar & mId;
    ar & subpartLimit;
    ar & onRayIntersectionMode;
    ar & onRayIntersectionArgument;
    ar & randomShift;
    ar & ladlut;
    ar & mOrigin;
    ar & mRotation;
    ar & mScale;
    ar & forceOnGround;
  }

public:
  // ***  TYPE ENUM  *** //
  // ******************** //
  /**
   * @brief Specify the type of scene part / object.
   * By default, the ScenePart is a static object.
   */
  enum ObjectType
  {
    STATIC_OBJECT,
    DYN_OBJECT,
    DYN_MOVING_OBJECT
  };
  /**
   * @brief Specify the type of primitive used to build the scene part
   * By default, the ScenePart is not build from primitives (None)
   * @see ScenePart::geometryType
   */
  enum GeometryType
  {
    NONE,
    TRIANGLE,
    VOXEL,
    DETAILED_VOXEL
  };
  // ***  ATTRIBUTES  *** //
  // ******************** //
  /**
   * @brief The type of primitive used to build the scene part
   * @see ScenePart::GeometryType
   */
  GeometryType geometryType;
  /**
   * @brief Transient storage used to expose vertex pointer views for APIs that
   * still expect `std::vector<Vertex*>`.
   */
  mutable std::vector<Vertex> mVertexScratch;
  /**
   * @brief The centroid of the scene part
   */
  arma::colvec centroid;
  /**
   * @brief The axis aligned bounding box defining scene part boundaries
   */
  std::shared_ptr<AABB> bound = nullptr;

  /**
   * @brief Identifier for the scene part
   */
  std::string mId = "";
  /**
   * @brief Vector specifying the limit of a subpart as the index of first
   *  element of next subpart.
   *
   * This vector makes sense when a scene part is built from multiple
   *  objects, so each one must be placed on its own scene part.
   *
   * The ith subpart is defined inside interval \f$[u[i-1], u[i])\f$, where
   *  \f$u\f$ is the subpartLimit vector. For the first case, the interval
   *  is defined as \f$[0, u[i])\f$
   * Having \f$|u| = 1\f$ means there is only one scene part
   */
  std::vector<size_t> subpartLimit;
  /**
   * @brief Specify the handling mode for ray intersections
   */
  std::string onRayIntersectionMode = "";
  /**
   * @brief Specify the extra value to be used for ray intersection handling
   * computation, when needed (depends on mode).
   */
  double onRayIntersectionArgument = 0.0;
  /**
   * @brief Specify if apply random shift to the scene part (true)
   * or not (false, by default)
   */
  bool randomShift = false;

  /**
   * @brief Look-up table for leaf angle distribution
   */
  std::shared_ptr<LadLut> ladlut = nullptr;

  /**
   * @brief Specify the origin for the scene part
   */
  glm::dvec3 mOrigin = glm::dvec3(0, 0, 0);
  /**
   * @brief Specify the rotation for the scene part
   */
  Rotation mRotation = Rotation(glm::dvec3(1, 0, 0), 0);
  /**
   * @brief Specify the scale for the scene part
   */
  double mScale = 1;
  /**
   * @brief Force the scene part to be on ground if \f$\neq 0\f$, do nothing
   *  if \f$= 0\f$.
   *
   * The force on ground process assumes the ground can be reduced to a
   *  plane and uses a search process:
   * <br/><b>-1</b> : Force on ground with optimum vertex, it is the one
   *  which is closest to the ground plane. Optimum solution with respect
   *  to ground plane is guaranteed but it is the most computationally
   *  expensive mode
   * <br/><b>0</b> : Dont force on ground at all
   * <br/><b>1</b> : Uses the minimum \f$z\f$ vertex to calculate to ground
   *  translation. It is the least computationally expensive mode
   * <br/><b>>1</b> : Uses a search process considering as many search steps
   *  as given. It allows to approximate the optimum vertex while maintaining
   *  computational cost inside desired boundaries
   *
   * NOTICE forceOnGround will not force the object to be on ground if it
   *  is a dynamic object which moves around the scene. It only assures the
   *  object will be INITIALLY placed at ground level
   */
  int forceOnGround = 0;

  OGRSpatialReference* mCrs = nullptr;
  OGREnvelope* mEnv = nullptr;

  /**
   * @brief The handler for scene parts that must swap its geometry for
   *  different simulation runs (repetitions).
   * @see SwapOnRepeatHandler
   * @see ScenePart::getSwapOnRepeatHandler
   * @see SimulationPlayer
   */
  std::shared_ptr<SwapOnRepeatHandler> sorh = nullptr;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Default constructor for a scene part
   * @param shallowGeometry If true, primitives pointers will be the same
   *  for the copy and for the source. If false, then primitives for the
   *  copy will be cloned so they are at different memory regions. Notice
   *  that a shallow copy of the primitives implies that the primitives in
   *  the copy will have the source as their reference scene part. Use this
   *  mode (true) with caution.
   */
  ScenePart()
    : geometryType(GeometryType::NONE)
  {
  }
  ScenePart(ScenePart const& sp, bool const shallowGeometry = false);
  virtual ~ScenePart() {}
  /**
   * @brief Polymorphic clone for scene parts.
   */
  virtual std::shared_ptr<ScenePart> clone(bool shallowGeometry = false) const;

  // ***  COPY / MOVE OPERATORS  *** //
  // ******************************* //
  /**
   * @brief Copy assigment operator.
   */
  ScenePart& operator=(const ScenePart& rhs);

  // ***   M E T H O D S   *** //
  // ************************* //
  /**
   * @brief Add the primitives of a WavefrontObj to the ScenePart
   * @param obj Pointer to a loaded OBJ
   */
  void addObj(WavefrontObj* obj);
  /**
   * @brief Obtain all vertices in the scene part
   * @return All vertices in the scene part
   */
  std::vector<Vertex*> getAllVertices() const;

  /**
   * @brief Smooth normals for each vertex computing the mean for each
   * triangle using it
   */
  void smoothVertexNormals();

  /**
   * @brief Split each subpart into a different scene part, with the first
   *  one corresponding to this scene part
   * @param splitParts Optional output vector receiving generated scene parts
   *  created from subparts (excluding this instance).
   * @see subpartLimit
   * @return True when split was successfully performed, false otherwise
   */
  bool splitSubparts(
    std::vector<std::shared_ptr<ScenePart>>* splitParts = nullptr);

  /**
   * @brief Compute the default centroid for the scene part as the midrange
   *  point and its boundaries too
   *
   * Let \f$P = \left\{p_1, \ldots, p_m\right\}\f$ be the set of
   *  points/vertices defining the scene part, where
   *  \f$\forall p_i \in P,\, p_i=\left(p_{ix}, p_{iy}, p_{iz}\right)\f$. So
   *  \f$\vec{p}_{x}\f$ is the vector containing the \f$m\f$
   *  \f$x\f$-coordinates, \f$\vec{p}_{y}\f$ is the vector containing the
   *  \f$m\f$ \f$y\f$-coordinates and \f$\vec{p}_{z}\f$ is the vector
   *  containing the \f$m\f$ \f$z\f$-cordinates. Now, let \f$o\f$ be the
   *  centroid of the scene part, so it can be defined as
   *
   * \f[
   * \begin{array}{lll}
   *  o &=& \frac{1}{2} \left(
   *      \min\left(\vec{p}_x\right)+\max\left(\vec{p}_{x}\right),
   *      \min\left(\vec{p}_y\right)+\max\left(\vec{p}_{y}\right),
   *      \min\left(\vec{p}_z\right)+\max\left(\vec{p}_{z}\right)
   *  \right) \\
   *  &=& \frac{1}{2} \left(\frac{a+b}{2}\right)
   * \end{array}
   * \f]
   *
   * The boundaries of the scene part are defined as the set of the min and
   *  max vertices respectively \f$\{a, b\}\f$ where:
   *
   * \f[
   * \begin{array}{lll}
   *  a &=& \left(
   *      \min\left(\vec{p}_x\right),
   *      \min\left(\vec{p}_y\right),
   *      \min\left(\vec{p}_z\right)
   *  \right) \\
   *  b &=& \left(
   *      \max\left(\vec{p}_x\right),
   *      \max\left(\vec{p}_y\right),
   *      \max\left(\vec{p}_z\right)
   *  \right)
   * \end{array}
   * \f]
   *
   * @param computeBound True to specify boundaries must be computed, false
   *  to avoid it. They are already necessary to compute the centroid, so
   *  if they are needed, calling this function with computeBound=true will
   *  store them with no extra computations but the ones required to build
   *  the AABB object itself
   *
   * @see ScenePart::centroid
   * @see AABB
   */
  void computeCentroid(bool const computeBound = false);

  /*
   * @brief Release all the resources that belong to the scene part. Note
   *  that calling release implies that the scene part can no longer be
   *  used.
   */
  virtual void release();

  // ***  INDEX-BASED GEOMETRY API  *** //
  // ********************************** //
  /**
   * @brief Number of geometry elements owned by this scene part.
   */
  virtual std::size_t geometryCount() const;
  /**
   * @brief Type of geometry elements owned by this scene part.
   */
  virtual GeometryType geometryTypeOf() const;
  /**
   * @brief Compute and optionally cache the bounding box of the scene part.
   */
  virtual std::shared_ptr<AABB> computeBound();
  /**
   * @brief Geometry element centroid by index.
   */
  virtual glm::dvec3 geometryCentroid(std::size_t index) const;
  /**
   * @brief Geometry element ray intersection distance by index.
   */
  virtual double geometryRayIntersectionDistance(
    std::size_t index,
    glm::dvec3 const& rayOrigin,
    glm::dvec3 const& rayDir) const;
  /**
   * @brief Geometry element ray intersection interval(s) by index.
   */
  virtual std::vector<double> geometryRayIntersection(
    std::size_t index,
    glm::dvec3 const& rayOrigin,
    glm::dvec3 const& rayDir) const;
  /**
   * @brief Geometry element incidence angle by index.
   */
  virtual double geometryIncidenceAngle(
    std::size_t index,
    glm::dvec3 const& rayOrigin,
    glm::dvec3 const& rayDir,
    glm::dvec3 const& intersectionPoint) const;
  /**
   * @brief Geometry element vertex count by index.
   */
  virtual std::size_t geometryVertexCount(std::size_t index) const;
  /**
   * @brief Number of dynamic vertices used by motion updates.
   */
  virtual std::size_t geometryDynamicVertexCount(std::size_t index) const;
  /**
   * @brief Dynamic vertex position by geometry/vertex indices.
   */
  virtual glm::dvec3 geometryDynamicVertexPosition(
    std::size_t geometryIndex,
    std::size_t vertexIndex) const;
  /**
   * @brief Dynamic vertex normal by geometry/vertex indices.
   */
  virtual glm::dvec3 geometryDynamicVertexNormal(std::size_t geometryIndex,
                                                 std::size_t vertexIndex) const;
  /**
   * @brief Set dynamic vertex position by geometry/vertex indices.
   */
  virtual void setGeometryDynamicVertexPosition(std::size_t geometryIndex,
                                                std::size_t vertexIndex,
                                                glm::dvec3 const& position);
  /**
   * @brief Set dynamic vertex normal by geometry/vertex indices.
   */
  virtual void setGeometryDynamicVertexNormal(std::size_t geometryIndex,
                                              std::size_t vertexIndex,
                                              glm::dvec3 const& normal);
  /**
   * @brief Geometry element ground z offset by index.
   */
  virtual double geometryGroundZOffset(std::size_t index) const;
  /**
   * @brief Whether geometry can compute sigma via LAD LUT by index.
   */
  virtual bool geometryCanComputeSigmaWithLadLut(std::size_t index) const;
  /**
   * @brief Compute sigma via LAD LUT by index.
   */
  virtual double geometryComputeSigmaWithLadLut(
    std::size_t index,
    glm::dvec3 const& direction) const;
  /**
   * @brief Whether geometry can handle intersections by index.
   */
  virtual bool geometryCanHandleIntersections(std::size_t index) const;
  /**
   * @brief Geometry element AABB by index.
   */
  virtual std::shared_ptr<AABB> geometryAABB(std::size_t index) const;
  /**
   * @brief Handle geometry-ray intersection by index.
   */
  virtual IntersectionHandlingResult geometryOnRayIntersection(
    std::size_t index,
    NoiseSource<double>& uniformNoiseSource,
    glm::dvec3& rayDirection,
    glm::dvec3 const& insideIntersectionPoint,
    glm::dvec3 const& outsideIntersectionPoint,
    double rayIntensity) const;
  /**
   * @brief Geometry element material accessor by index.
   */
  virtual std::shared_ptr<Material> geometryMaterial(std::size_t index) const;
  /**
   * @brief Geometry element material mutator by index.
   */
  virtual void setGeometryMaterial(std::size_t index,
                                   std::shared_ptr<Material> material);
  /**
   * @brief Rebind primitive owner pointers to the given scene part.
   */
  virtual void bindGeometryOwners(std::shared_ptr<ScenePart> const& owner);
  /**
   * @brief Rebind owner pointers only for primitives with no owner.
   */
  virtual void bindUnownedGeometryOwners(
    std::shared_ptr<ScenePart> const& owner);
  /**
   * @brief Rotate primitive by index.
   */
  virtual void geometryRotate(std::size_t index, Rotation& rotation);
  /**
   * @brief Scale primitive by index.
   */
  virtual void geometryScale(std::size_t index, double factor);
  /**
   * @brief Translate primitive by index.
   */
  virtual void geometryTranslate(std::size_t index, glm::dvec3 const& shift);
  /**
   * @brief Update primitive by index.
   */
  virtual void geometryUpdate(std::size_t index);
  /**
   * @brief Trigger post-load primitive hook by index.
   */
  virtual void geometryOnFinishLoading(std::size_t index,
                                       NoiseSource<double>& uniformNoiseSource);
  /**
   * @brief Delete all owned primitive objects and clear storage.
   */
  virtual void deleteGeometries();
  /**
   * @brief Clear primitive storage without deleting pointed objects.
   */
  virtual void clearGeometryStorage();
  /**
   * @brief Clear owner pointers for all primitives.
   */
  virtual void unbindGeometryOwners();

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @brief Obtain the centroid of the scene part
   * @return Scene part centroid
   * @see ScenePart::centroid
   */
  inline arma::colvec getCentroid() const { return centroid; }
  /**
   * @brief Set the centroid of the scene part
   * @param centroid New centroid for the scene part
   * @see ScenePart::centroid
   */
  inline void setCentroid(arma::colvec centroid) { this->centroid = centroid; }

  /**
   * @brief Obtain the ID of the scene part
   * @return Scene part ID
   * @see ScenePart::mId
   */
  inline std::string const& getId() const { return mId; }
  /**
   * @brief Set the ID of the scene part
   * @param id Scene part ID
   * @see ScenePart::id
   */
  inline void setId(const std::string& id) { this->mId = id; }
  /**
   * @brief Obtain the object type of the scene part
   * @return Object type of the scene part
   * @see ScenePart::ObjectType
   */
  virtual ObjectType getType() const { return ObjectType::STATIC_OBJECT; }
  virtual GeometryType getGeometryType() const { return geometryType; }

  /**
   * @brief Obtain the swap on repeat handler of the scene part.
   * @return Swap on repeat handler of the scene part.
   * @see ScenePart::sorh
   */
  inline std::shared_ptr<SwapOnRepeatHandler> getSwapOnRepeatHandler()
  {
    return sorh;
  }

  /**
   * @brief Check whether the scene part is null, according to the underlying
   *  swap on repeat handler.
   * @return True if the scene part is null, false otherwise.
   * @see SwapOnRepeatHandler
   * @see SwapOnRepeatHandler::null
   * @see SwapOnRepeatHandler::isNull
   */
  inline bool isNull() const
  {
    return (sorh == nullptr) ? false : sorh->isNull();
  }

  // ***  STATIC METHODS  *** //
  // ************************ //
  /**
   * @brief Compute the transformations for each primitive in the scene part.
   *  The transformations are typically specified through assetloading
   *  filters.
   * @param sp The ScenePart whose transformations must be computed.
   * @param holistic Whether to use an holistic approach for all the vertices
   *  (e.g., for points loaded from point clouds as input).
   * @see XmlSceneLoader
   * @see XmlSceneLoader::digestScenePart
   * @see SwapOnRepeatHandler
   * @see SimulationPlayer
   */
  static void computeTransformations(std::shared_ptr<ScenePart> sp,
                                     bool const holistic = false);
};

BOOST_CLASS_VERSION(ScenePart, 2)
