#include <boost/lexical_cast.hpp>
#include <boost/lexical_cast/bad_lexical_cast.hpp>

#include <algorithm>

#include <AABB.h>
#include <DetailedVoxel.h>
#include <Primitive.h>
#include <ScenePart.h>
#include <Triangle.h>
#include <Voxel.h>
#include <WavefrontObj.h>
#include <scene/primitives/PrimitiveViews.h>
#include <util/logger/logging.hpp>

#include <glm/glm.hpp>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
ScenePart::ScenePart()
  : primitiveType(PrimitiveType::NONE)
{
}

ScenePart::ScenePart(ScenePart const& sp, bool const shallowPrimitives)
{
  this->centroid = sp.centroid;
  this->bound = sp.bound;
  this->mId = sp.mId;
  this->onRayIntersectionMode = sp.onRayIntersectionMode;
  this->onRayIntersectionArgument = sp.onRayIntersectionArgument;
  this->randomShift = sp.randomShift;
  this->mOrigin = glm::dvec3(sp.mOrigin);
  this->mRotation = Rotation(sp.mRotation);
  this->mScale = sp.mScale;
  this->forceOnGround = sp.forceOnGround;
  this->mCrs = nullptr; // TODO Copy this too
  this->mEnv = nullptr; // TODO Copy this too

  this->primitiveType = sp.primitiveType;
  if (sp.triangles.size() == 0 && sp.voxels.size() == 0 &&
      !sp.mPrimitives.empty()) {
    this->mPrimitives = sp.mPrimitives;
    buildBulkFromPrimitives();
    this->mPrimitives.clear();
  } else {
    this->triangles = sp.triangles;
    this->voxels = sp.voxels;
    this->detailed_voxels = sp.detailed_voxels;
  }
  clearPrimitiveCache();
  buildPrimitiveViewsFromBulk();

  this->subpartLimit = sp.subpartLimit;
}

ScenePart::~ScenePart() = default;

// ***  COPY / MOVE OPERATORS  *** //
// ******************************* //
ScenePart&
ScenePart::operator=(const ScenePart& rhs)
{
  this->centroid = rhs.centroid;
  this->bound = rhs.bound;
  this->mId = rhs.mId;
  this->onRayIntersectionMode = rhs.onRayIntersectionMode;
  this->onRayIntersectionArgument = rhs.onRayIntersectionArgument;
  this->randomShift = rhs.randomShift;
  this->mOrigin = glm::dvec3(rhs.mOrigin);
  this->mRotation = Rotation(rhs.mRotation);
  this->mScale = rhs.mScale;
  this->forceOnGround = rhs.forceOnGround;
  this->mCrs = nullptr; // TODO Copy this too
  this->mEnv = nullptr; // TODO Copy this too

  this->primitiveType = rhs.primitiveType;
  if (rhs.triangles.size() == 0 && rhs.voxels.size() == 0 &&
      !rhs.mPrimitives.empty()) {
    this->mPrimitives = rhs.mPrimitives;
    buildBulkFromPrimitives();
    this->mPrimitives.clear();
  } else {
    this->triangles = rhs.triangles;
    this->voxels = rhs.voxels;
    this->detailed_voxels = rhs.detailed_voxels;
  }
  clearPrimitiveCache();
  buildPrimitiveViewsFromBulk();

  this->subpartLimit = rhs.subpartLimit;

  return *this;
}

// ***  M E T H O D S  *** //
// *********************** //

void
ScenePart::addObj(WavefrontObj* obj)
{
  std::stringstream ss;
  ss << "Adding primitive to Scenepart ...";
  logging::DEBUG(ss.str());
  ss.str("");
  if (obj == nullptr)
    return;

  primitiveType = PrimitiveType::TRIANGLE;
  size_t oldNumTriangles = triangles.size();

  triangles.vertices.insert(triangles.vertices.end(),
                            obj->triangles.vertices.begin(),
                            obj->triangles.vertices.end());
  triangles.face_normal.insert(triangles.face_normal.end(),
                               obj->triangles.face_normal.begin(),
                               obj->triangles.face_normal.end());
  triangles.e1.insert(
    triangles.e1.end(), obj->triangles.e1.begin(), obj->triangles.e1.end());
  triangles.e2.insert(
    triangles.e2.end(), obj->triangles.e2.begin(), obj->triangles.e2.end());
  triangles.v0.insert(
    triangles.v0.end(), obj->triangles.v0.begin(), obj->triangles.v0.end());
  triangles.eps.insert(
    triangles.eps.end(), obj->triangles.eps.begin(), obj->triangles.eps.end());
  triangles.aabb_min.insert(triangles.aabb_min.end(),
                            obj->triangles.aabb_min.begin(),
                            obj->triangles.aabb_min.end());
  triangles.aabb_max.insert(triangles.aabb_max.end(),
                            obj->triangles.aabb_max.begin(),
                            obj->triangles.aabb_max.end());
  triangles.materials.insert(triangles.materials.end(),
                             obj->triangles.materials.begin(),
                             obj->triangles.materials.end());

  ss << "# new primitives added: " << triangles.size() - oldNumTriangles;
  logging::DEBUG(ss.str());
}

std::vector<Vertex*>
ScenePart::getAllVertices() const
{
  std::vector<Vertex*> allPos;
  if (primitiveType == PrimitiveType::TRIANGLE) {
    allPos.reserve(triangles.vertices.size());
    for (Vertex const& v : triangles.vertices) {
      allPos.push_back(const_cast<Vertex*>(&v));
    }
  } else if (primitiveType == PrimitiveType::VOXEL) {
    allPos.reserve(voxels.centers.size());
    for (Vertex const& v : voxels.centers) {
      allPos.push_back(const_cast<Vertex*>(&v));
    }
  } else {
    for (Primitive* p : mPrimitives) {
      for (size_t i = 0; i < p->getNumVertices(); i++) {
        allPos.push_back(p->getVertices() + i);
      }
    }
  }
  return allPos;
}

void
ScenePart::smoothVertexNormals()
{
  if (primitiveType != PrimitiveType::TRIANGLE)
    return;

  std::map<Vertex*, std::vector<PrimitiveIndex>> vtmap;
  size_t const n = triangles.size();
  for (PrimitiveIndex i = 0; i < n; ++i) {
    Vertex* verts = &triangles.vertices[3 * i];
    for (int j = 0; j < 3; ++j) {
      vtmap[verts + j].push_back(i);
    }
  }

  for (auto& it : vtmap) {
    Vertex* v = it.first;
    std::vector<PrimitiveIndex>& tris = it.second;
    glm::dvec3 normal(0.0, 0.0, 0.0);
    for (PrimitiveIndex i : tris) {
      glm::dvec3 const e1 = triangles.e1[i];
      glm::dvec3 const e2 = triangles.e2[i];
      glm::dvec3 face = glm::cross(e1, e2);
      double const len = glm::length(face);
      if (len > 0.0)
        face = face / len;
      normal += face;
    }
    if (glm::length(normal) > 0.0)
      normal = glm::normalize(normal);
    v->normal = normal;
  }
}

std::vector<std::shared_ptr<ScenePart>>
ScenePart::splitSubparts()
{
  std::vector<std::shared_ptr<ScenePart>> newParts;
  size_t n = subpartLimit.size();
  if (n <= 1)
    return newParts; // There is no need to do splits

  // Prepare incremental ID
  int start = -1;
  try {
    start = boost::lexical_cast<int>(mId);
  } catch (boost::bad_lexical_cast& blcex) {
    std::stringstream ss;
    ss << "Could not update subpart ID from \"" << mId << "\".\n "
       << "Thus, splitting subparts is aborted";
    logging::WARN(ss.str());
    return newParts;
  }

  // Do splits
  for (size_t i = 1; i < n; ++i) {
    std::shared_ptr<ScenePart> newPart = std::make_shared<ScenePart>();
    newPart->onRayIntersectionMode = onRayIntersectionMode;
    newPart->onRayIntersectionArgument = onRayIntersectionArgument;
    newPart->randomShift = randomShift;
    if (ladlut == nullptr)
      newPart->ladlut = nullptr;
    else
      newPart->ladlut = std::make_shared<LadLut>(*ladlut);
    newPart->mOrigin = mOrigin;
    newPart->mRotation = mRotation;
    newPart->mScale = mScale;
    newPart->forceOnGround = forceOnGround;
    newPart->primitiveType = primitiveType;
    size_t const startIndex = subpartLimit[i - 1];
    size_t const endIndex = subpartLimit[i];

    if (primitiveType == PrimitiveType::TRIANGLE) {
      size_t const vStart = 3 * startIndex;
      size_t const vEnd = 3 * endIndex;
      newPart->triangles.vertices.assign(triangles.vertices.begin() + vStart,
                                         triangles.vertices.begin() + vEnd);
      newPart->triangles.face_normal.assign(
        triangles.face_normal.begin() + startIndex,
        triangles.face_normal.begin() + endIndex);
      newPart->triangles.e1.assign(triangles.e1.begin() + startIndex,
                                   triangles.e1.begin() + endIndex);
      newPart->triangles.e2.assign(triangles.e2.begin() + startIndex,
                                   triangles.e2.begin() + endIndex);
      newPart->triangles.v0.assign(triangles.v0.begin() + startIndex,
                                   triangles.v0.begin() + endIndex);
      newPart->triangles.eps.assign(triangles.eps.begin() + startIndex,
                                    triangles.eps.begin() + endIndex);
      newPart->triangles.aabb_min.assign(triangles.aabb_min.begin() +
                                           startIndex,
                                         triangles.aabb_min.begin() + endIndex);
      newPart->triangles.aabb_max.assign(triangles.aabb_max.begin() +
                                           startIndex,
                                         triangles.aabb_max.begin() + endIndex);
      newPart->triangles.materials.assign(
        triangles.materials.begin() + startIndex,
        triangles.materials.begin() + endIndex);
    } else if (primitiveType == PrimitiveType::VOXEL) {
      newPart->voxels.centers.assign(voxels.centers.begin() + startIndex,
                                     voxels.centers.begin() + endIndex);
      newPart->voxels.half_size.assign(voxels.half_size.begin() + startIndex,
                                       voxels.half_size.begin() + endIndex);
      newPart->voxels.num_points.assign(voxels.num_points.begin() + startIndex,
                                        voxels.num_points.begin() + endIndex);
      newPart->voxels.r.assign(voxels.r.begin() + startIndex,
                               voxels.r.begin() + endIndex);
      newPart->voxels.g.assign(voxels.g.begin() + startIndex,
                               voxels.g.begin() + endIndex);
      newPart->voxels.b.assign(voxels.b.begin() + startIndex,
                               voxels.b.begin() + endIndex);
      newPart->voxels.color.assign(voxels.color.begin() + startIndex,
                                   voxels.color.begin() + endIndex);
      newPart->voxels.aabb_min.assign(voxels.aabb_min.begin() + startIndex,
                                      voxels.aabb_min.begin() + endIndex);
      newPart->voxels.aabb_max.assign(voxels.aabb_max.begin() + startIndex,
                                      voxels.aabb_max.begin() + endIndex);
      newPart->voxels.materials.assign(voxels.materials.begin() + startIndex,
                                       voxels.materials.begin() + endIndex);

      if (!detailed_voxels.present.empty()) {
        newPart->detailed_voxels.present.assign(
          detailed_voxels.present.begin() + startIndex,
          detailed_voxels.present.begin() + endIndex);
        newPart->detailed_voxels.int_values.assign(
          detailed_voxels.int_values.begin() + startIndex,
          detailed_voxels.int_values.begin() + endIndex);
        newPart->detailed_voxels.double_values.assign(
          detailed_voxels.double_values.begin() + startIndex,
          detailed_voxels.double_values.begin() + endIndex);
        newPart->detailed_voxels.max_pad.assign(
          detailed_voxels.max_pad.begin() + startIndex,
          detailed_voxels.max_pad.begin() + endIndex);
      }
    }

    newPart->buildPrimitiveViewsFromBulk();
    newPart->mId = std::to_string(start + i);
    newParts.push_back(newPart);
  }

  // Remove splitted primitives from current part
  if (primitiveType == PrimitiveType::TRIANGLE) {
    size_t const keep = subpartLimit[0];
    triangles.vertices.erase(triangles.vertices.begin() + 3 * keep,
                             triangles.vertices.end());
    triangles.face_normal.erase(triangles.face_normal.begin() + keep,
                                triangles.face_normal.end());
    triangles.e1.erase(triangles.e1.begin() + keep, triangles.e1.end());
    triangles.e2.erase(triangles.e2.begin() + keep, triangles.e2.end());
    triangles.v0.erase(triangles.v0.begin() + keep, triangles.v0.end());
    triangles.eps.erase(triangles.eps.begin() + keep, triangles.eps.end());
    triangles.aabb_min.erase(triangles.aabb_min.begin() + keep,
                             triangles.aabb_min.end());
    triangles.aabb_max.erase(triangles.aabb_max.begin() + keep,
                             triangles.aabb_max.end());
    triangles.materials.erase(triangles.materials.begin() + keep,
                              triangles.materials.end());
  } else if (primitiveType == PrimitiveType::VOXEL) {
    size_t const keep = subpartLimit[0];
    voxels.centers.erase(voxels.centers.begin() + keep, voxels.centers.end());
    voxels.half_size.erase(voxels.half_size.begin() + keep,
                           voxels.half_size.end());
    voxels.num_points.erase(voxels.num_points.begin() + keep,
                            voxels.num_points.end());
    voxels.r.erase(voxels.r.begin() + keep, voxels.r.end());
    voxels.g.erase(voxels.g.begin() + keep, voxels.g.end());
    voxels.b.erase(voxels.b.begin() + keep, voxels.b.end());
    voxels.color.erase(voxels.color.begin() + keep, voxels.color.end());
    voxels.aabb_min.erase(voxels.aabb_min.begin() + keep,
                          voxels.aabb_min.end());
    voxels.aabb_max.erase(voxels.aabb_max.begin() + keep,
                          voxels.aabb_max.end());
    voxels.materials.erase(voxels.materials.begin() + keep,
                           voxels.materials.end());

    if (!detailed_voxels.present.empty()) {
      detailed_voxels.present.erase(detailed_voxels.present.begin() + keep,
                                    detailed_voxels.present.end());
      detailed_voxels.int_values.erase(detailed_voxels.int_values.begin() +
                                         keep,
                                       detailed_voxels.int_values.end());
      detailed_voxels.double_values.erase(
        detailed_voxels.double_values.begin() + keep,
        detailed_voxels.double_values.end());
      detailed_voxels.max_pad.erase(detailed_voxels.max_pad.begin() + keep,
                                    detailed_voxels.max_pad.end());
    }
  }
  buildPrimitiveViewsFromBulk();
  subpartLimit.clear();
  mId = std::to_string(start);

  /*
   * /!\  WARNING  /!\
   * The origin, as rotation and scale transformations, are applied before
   *  splitting. Hence, transformations are applied with respect to the
   *  original scene part.
   * Thus, each new scene part coming from splitting the original one stills
   *  considering the origin and transformation specifications of original
   *  scene part.
   * In consequence, they can be easily reverted. But, to apply
   *  transformations to new scene parts, their attributes such as origin
   *  should be updated to new primitive. Consider this when manipulating
   *  those subparts in future. The original purpose for this split was to
   *  have different hitObjectId for different components.
   */
  return newParts;
}

void
ScenePart::computeCentroid(bool const computeBound)
{
  // Find centroid coordinates
  double xmin = std::numeric_limits<double>::max();
  double xmax = std::numeric_limits<double>::lowest();
  double ymin = xmin, ymax = xmax, zmin = xmin, zmax = xmax;
  std::vector<Vertex*> vertices = getAllVertices();
  for (Vertex* vertex : vertices) {
    // Find centroid x coordinate
    double const x = vertex->getX();
    if (x < xmin)
      xmin = x;
    if (x > xmax)
      xmax = x;
    // Find centroid y coordinate
    double const y = vertex->getY();
    if (y < ymin)
      ymin = y;
    if (y > ymax)
      ymax = y;
    // Find centroid z coordinate
    double const z = vertex->getZ();
    if (z < zmin)
      zmin = z;
    if (z > zmax)
      zmax = z;
  }

  // Build the centroid
  centroid = arma::colvec(std::vector<double>({
    (xmin + xmax) / 2.0,
    (ymin + ymax) / 2.0,
    (zmin + zmax) / 2.0,
  }));

  // Build the bound
  if (computeBound) {
    bound = std::make_shared<AABB>(xmin, ymin, zmin, xmax, ymax, zmax);
  }
}

void
ScenePart::clearBulkData()
{
  triangles.clear();
  voxels.clear();
  detailed_voxels.clear();
}

void
ScenePart::clearPrimitiveCache()
{
  mPrimitives.clear();
  primitiveCache.reset();
}

static std::shared_ptr<ScenePart>
makeNonOwningShared(ScenePart* sp)
{
  return std::shared_ptr<ScenePart>(sp, [](ScenePart*) {});
}

void
ScenePart::buildPrimitiveViewsFromBulk()
{
  clearPrimitiveCache();
  primitiveCache = std::make_unique<ScenePartPrimitiveCache>();

  std::shared_ptr<ScenePart> partPtr = makeNonOwningShared(this);

  if (primitiveType == PrimitiveType::TRIANGLE) {
    std::size_t const n = triangles.size();
    primitiveCache->triangleViews.resize(n);
    mPrimitives.reserve(n);
    for (PrimitiveIndex i = 0; i < n; ++i) {
      primitiveCache->triangleViews[i] = TriangleView(this, i);
      TriangleView& view = primitiveCache->triangleViews[i];
      view.part = partPtr;
      if (i < triangles.materials.size())
        view.material = triangles.materials[i];
      mPrimitives.push_back(&view);
    }
    return;
  }

  if (primitiveType == PrimitiveType::VOXEL) {
    std::size_t const n = voxels.size();
    primitiveCache->voxelViews.resize(n);
    primitiveCache->detailedVoxelViews.resize(n);
    mPrimitives.reserve(n);
    for (PrimitiveIndex i = 0; i < n; ++i) {
      primitiveCache->voxelViews[i] = VoxelView(this, i);
      primitiveCache->detailedVoxelViews[i] = DetailedVoxelView(this, i);

      bool isDetailed =
        (i < detailed_voxels.present.size() && detailed_voxels.present[i] != 0);
      if (isDetailed) {
        DetailedVoxelView& view = primitiveCache->detailedVoxelViews[i];
        view.part = partPtr;
        if (i < voxels.materials.size())
          view.material = voxels.materials[i];
        mPrimitives.push_back(&view);
      } else {
        VoxelView& view = primitiveCache->voxelViews[i];
        view.part = partPtr;
        if (i < voxels.materials.size())
          view.material = voxels.materials[i];
        mPrimitives.push_back(&view);
      }
    }
  }
}

void
ScenePart::buildBulkFromPrimitives()
{
  clearBulkData();
  if (mPrimitives.empty())
    return;

  if (primitiveType == PrimitiveType::NONE) {
    size_t const numVertices = mPrimitives[0]->getNumVertices();
    primitiveType =
      (numVertices == 3) ? PrimitiveType::TRIANGLE : PrimitiveType::VOXEL;
  }

  if (primitiveType == PrimitiveType::TRIANGLE) {
    std::size_t const n = mPrimitives.size();

    for (Primitive* primitive : mPrimitives) {
      Triangle* t = dynamic_cast<Triangle*>(primitive);
      if (t == nullptr) {
        logging::WARN(
          "ScenePart::buildBulkFromPrimitives found non-triangle primitive");
        clearBulkData();
        return;
      }

      Vertex* verts = t->getVertices();
      appendTriangleBulk(triangles, verts[0], verts[1], verts[2], t->material);
    }
    return;
  }

  if (primitiveType == PrimitiveType::VOXEL) {
    for (Primitive* primitive : mPrimitives) {
      Voxel* v = dynamic_cast<Voxel*>(primitive);
      if (v == nullptr) {
        logging::WARN(
          "ScenePart::buildBulkFromPrimitives found non-voxel primitive");
        clearBulkData();
        return;
      }

      appendVoxelBulk(voxels,
                      v->v,
                      v->halfSize,
                      v->numPoints,
                      v->r,
                      v->g,
                      v->b,
                      v->color,
                      v->material);

      DetailedVoxel* dv = dynamic_cast<DetailedVoxel*>(primitive);
      if (dv == nullptr) {
        detailed_voxels.present.push_back(0);
        detailed_voxels.int_values.emplace_back();
        detailed_voxels.double_values.emplace_back();
        detailed_voxels.max_pad.push_back(0.0);
        continue;
      }

      detailed_voxels.present.push_back(1);
      std::vector<int> int_values;
      int_values.reserve(dv->getNumberOfIntValues());
      for (std::size_t i = 0; i < dv->getNumberOfIntValues(); ++i)
        int_values.push_back(dv->getIntValue(i));
      detailed_voxels.int_values.push_back(std::move(int_values));

      std::vector<double> double_values;
      double_values.reserve(dv->getNumberOfDoubleValues());
      for (std::size_t i = 0; i < dv->getNumberOfDoubleValues(); ++i)
        double_values.push_back(dv->getDoubleValue(i));
      detailed_voxels.double_values.push_back(std::move(double_values));

      detailed_voxels.max_pad.push_back(dv->getMaxPad());
    }
  }
}

void
ScenePart::buildPrimitivesFromBulk(bool const clearExisting)
{
  if (clearExisting) {
    for (Primitive* p : mPrimitives)
      if (!isPrimitiveView(p))
        delete p;
    mPrimitives.clear();
  }

  if (primitiveType == PrimitiveType::TRIANGLE) {
    if (triangles.vertices.size() % 3 != 0) {
      logging::WARN("ScenePart::buildPrimitivesFromBulk triangle vertex count "
                    "is not a multiple of 3");
      return;
    }

    std::size_t const n = triangles.vertices.size() / 3;
    mPrimitives.reserve(mPrimitives.size() + n);
    for (std::size_t i = 0; i < n; ++i) {
      Vertex v0 = triangles.vertices[3 * i + 0];
      Vertex v1 = triangles.vertices[3 * i + 1];
      Vertex v2 = triangles.vertices[3 * i + 2];
      Triangle* t = new Triangle(v0, v1, v2);
      if (i < triangles.materials.size())
        t->material = triangles.materials[i];
      mPrimitives.push_back(t);
    }
    return;
  }

  if (primitiveType == PrimitiveType::VOXEL) {
    std::size_t const n = voxels.centers.size();
    mPrimitives.reserve(mPrimitives.size() + n);
    for (std::size_t i = 0; i < n; ++i) {
      bool const isDetailed =
        (i < detailed_voxels.present.size() && detailed_voxels.present[i] != 0);
      Primitive* p = nullptr;

      if (isDetailed) {
        std::vector<int> int_values;
        std::vector<double> double_values;
        if (i < detailed_voxels.int_values.size())
          int_values = detailed_voxels.int_values[i];
        if (i < detailed_voxels.double_values.size())
          double_values = detailed_voxels.double_values[i];

        DetailedVoxel* dv = new DetailedVoxel(voxels.centers[i].pos.x,
                                              voxels.centers[i].pos.y,
                                              voxels.centers[i].pos.z,
                                              voxels.half_size[i],
                                              std::move(int_values),
                                              std::move(double_values));
        if (i < detailed_voxels.max_pad.size())
          dv->setMaxPad(detailed_voxels.max_pad[i]);
        p = dv;
      } else {
        p = new Voxel(voxels.centers[i].pos.x,
                      voxels.centers[i].pos.y,
                      voxels.centers[i].pos.z,
                      voxels.half_size[i]);
      }

      Voxel* v = dynamic_cast<Voxel*>(p);
      if (v != nullptr) {
        v->v = voxels.centers[i];
        if (i < voxels.half_size.size())
          v->halfSize = voxels.half_size[i];
        if (i < voxels.num_points.size())
          v->numPoints = voxels.num_points[i];
        if (i < voxels.r.size())
          v->r = voxels.r[i];
        if (i < voxels.g.size())
          v->g = voxels.g[i];
        if (i < voxels.b.size())
          v->b = voxels.b[i];
        if (i < voxels.color.size())
          v->color = voxels.color[i];
        v->update();
      }

      if (i < voxels.materials.size())
        p->material = voxels.materials[i];
      mPrimitives.push_back(p);
    }
  }
}

void
ScenePart::updateTriangleBulk(PrimitiveIndex index)
{
  std::size_t const base = 3 * index;
  if (base + 2 >= triangles.vertices.size())
    return;

  Vertex* verts = &triangles.vertices[base];
  glm::dvec3 v0 = verts[0].pos;
  glm::dvec3 e1 = verts[1].pos - v0;
  glm::dvec3 e2 = verts[2].pos - v0;
  glm::dvec3 normal = glm::cross(e1, e2);
  double const len = glm::length(normal);
  if (len > 0.0)
    normal = normal / len;

  if (index >= triangles.v0.size())
    triangles.v0.resize(index + 1);
  if (index >= triangles.e1.size())
    triangles.e1.resize(index + 1);
  if (index >= triangles.e2.size())
    triangles.e2.resize(index + 1);
  if (index >= triangles.face_normal.size())
    triangles.face_normal.resize(index + 1);
  if (index >= triangles.eps.size())
    triangles.eps.resize(index + 1, 0.0000001);
  if (index >= triangles.aabb_min.size())
    triangles.aabb_min.resize(index + 1);
  if (index >= triangles.aabb_max.size())
    triangles.aabb_max.resize(index + 1);

  triangles.v0[index] = v0;
  triangles.e1[index] = e1;
  triangles.e2[index] = e2;
  triangles.face_normal[index] = normal;

  double minX =
    std::min(std::min(verts[0].getX(), verts[1].getX()), verts[2].getX());
  double minY =
    std::min(std::min(verts[0].getY(), verts[1].getY()), verts[2].getY());
  double minZ =
    std::min(std::min(verts[0].getZ(), verts[1].getZ()), verts[2].getZ());
  double maxX =
    std::max(std::max(verts[0].getX(), verts[1].getX()), verts[2].getX());
  double maxY =
    std::max(std::max(verts[0].getY(), verts[1].getY()), verts[2].getY());
  double maxZ =
    std::max(std::max(verts[0].getZ(), verts[1].getZ()), verts[2].getZ());

  triangles.aabb_min[index] = glm::dvec3(minX, minY, minZ);
  triangles.aabb_max[index] = glm::dvec3(maxX, maxY, maxZ);
}

void
ScenePart::updateVoxelBulk(PrimitiveIndex index)
{
  if (index >= voxels.centers.size() || index >= voxels.half_size.size())
    return;
  if (index >= voxels.aabb_min.size())
    voxels.aabb_min.resize(index + 1);
  if (index >= voxels.aabb_max.size())
    voxels.aabb_max.resize(index + 1);
  double halfSize = voxels.half_size[index];
  glm::dvec3 hs(halfSize, halfSize, halfSize);
  voxels.aabb_min[index] = voxels.centers[index].pos - hs;
  voxels.aabb_max[index] = voxels.centers[index].pos + hs;
}

void
ScenePart::updateBulk()
{
  if (primitiveType == PrimitiveType::TRIANGLE) {
    std::size_t const n = triangles.size();
    for (PrimitiveIndex i = 0; i < n; ++i) {
      updateTriangleBulk(i);
    }
  } else if (primitiveType == PrimitiveType::VOXEL) {
    std::size_t const n = voxels.size();
    for (PrimitiveIndex i = 0; i < n; ++i) {
      updateVoxelBulk(i);
    }
  }
}

void
ScenePart::appendPrimitiveRefs(std::vector<PrimitiveRef>& out) const
{
  if (primitiveType == PrimitiveType::TRIANGLE) {
    std::size_t const n = triangles.size();
    out.reserve(out.size() + n);
    for (PrimitiveIndex i = 0; i < n; ++i) {
      out.emplace_back(
        const_cast<ScenePart*>(this), PrimitiveType::TRIANGLE, i);
    }
    return;
  }

  if (primitiveType == PrimitiveType::VOXEL) {
    std::size_t const n = voxels.size();
    out.reserve(out.size() + n);
    for (PrimitiveIndex i = 0; i < n; ++i) {
      out.emplace_back(const_cast<ScenePart*>(this), PrimitiveType::VOXEL, i);
    }
  }
}

void
ScenePart::setPrimitives(std::vector<Primitive*> const& primitives)
{
  for (Primitive* p : mPrimitives) {
    if (!isPrimitiveView(p))
      delete p;
  }
  mPrimitives = primitives;
  if (mPrimitives.empty()) {
    clearBulkData();
    clearPrimitiveCache();
    primitiveType = PrimitiveType::NONE;
    return;
  }

  buildBulkFromPrimitives();

  for (Primitive* p : mPrimitives) {
    if (!isPrimitiveView(p))
      delete p;
  }
  mPrimitives.clear();

  buildPrimitiveViewsFromBulk();
}

void
ScenePart::computeTransformations(std::shared_ptr<ScenePart> sp,
                                  bool const holistic)
{
  // For all primitives, set reference to their scene part and transform:
  std::shared_ptr<ScenePart> nonOwning = makeNonOwningShared(sp.get());
  for (Primitive* p : sp->mPrimitives) {
    if (isPrimitiveView(p))
      p->part = nonOwning;
    else
      p->part = sp;
    p->rotate(sp->mRotation);
    if (holistic) {
      for (size_t i = 0; i < p->getNumVertices(); i++) {
        p->getVertices()[i].pos.x *= sp->mScale;
        p->getVertices()[i].pos.y *= sp->mScale;
        p->getVertices()[i].pos.z *= sp->mScale;
      }
    }
    p->scale(sp->mScale);
    p->translate(sp->mOrigin);
  }
}

void
ScenePart::release()
{
  for (Primitive* p : mPrimitives) {
    if (!isPrimitiveView(p))
      delete p;
  }
  mPrimitives.clear();
  clearPrimitiveCache();
  clearBulkData();
  if (sorh != nullptr) {
    for (Primitive* p : sorh->getBaselinePrimitives()) {
      if (!isPrimitiveView(p))
        delete p;
    }
    sorh->baseline = nullptr;
    sorh = nullptr;
  }
}
