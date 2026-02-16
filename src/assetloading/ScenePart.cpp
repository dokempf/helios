#include <boost/lexical_cast.hpp>
#include <boost/lexical_cast/bad_lexical_cast.hpp>

#include <AABB.h>
#include <DetailedVoxelScenePart.h>
#include <IntersectionHandlingResult.h>
#include <ScenePart.h>
#include <TriangleScenePart.h>
#include <VoxelScenePart.h>
#include <WavefrontObj.h>
#include <util/logger/logging.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <map>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
ScenePart::ScenePart(ScenePart const& sp, bool const)
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
  this->geometryType = sp.geometryType;
  this->subpartLimit = sp.subpartLimit;
}

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
  this->geometryType = rhs.geometryType;
  this->subpartLimit = rhs.subpartLimit;

  return *this;
}

std::shared_ptr<ScenePart>
ScenePart::clone(bool shallowGeometry) const
{
  return std::make_shared<ScenePart>(*this, shallowGeometry);
}

// ***  M E T H O D S  *** //
// *********************** //

void
ScenePart::addObj(WavefrontObj* obj)
{
  if (obj == nullptr) {
    return;
  }
  std::stringstream ss;
  ss << "Adding geometry to ScenePart ...";
  logging::DEBUG(ss.str());
  ss.str("");

  if (TriangleScenePart* trianglePart =
        dynamic_cast<TriangleScenePart*>(this)) {
    std::size_t const appendCount = obj->triangles.size();
    std::size_t const oldRows = trianglePart->vertices.n_rows;
    trianglePart->vertices.resize(oldRows + appendCount, 9);
    trianglePart->normals.resize(oldRows + appendCount, 9);
    trianglePart->materialIndex.resize(oldRows + appendCount);
    for (std::size_t i = 0; i < appendCount; ++i) {
      std::size_t const row = oldRows + i;
      WavefrontObj::TriangleRecord const& tri = obj->triangles[i];
      for (std::size_t j = 0; j < 3; ++j) {
        Vertex const& v = tri.verts[j];
        trianglePart->vertices(row, 3 * j) = v.pos.x;
        trianglePart->vertices(row, 3 * j + 1) = v.pos.y;
        trianglePart->vertices(row, 3 * j + 2) = v.pos.z;
        trianglePart->normals(row, 3 * j) = v.normal.x;
        trianglePart->normals(row, 3 * j + 1) = v.normal.y;
        trianglePart->normals(row, 3 * j + 2) = v.normal.z;
      }
      trianglePart->materialIndex(row) = 0;
      if (tri.material != nullptr) {
        trianglePart->setGeometryMaterial(row, tri.material);
      }
    }
    ss << "# new geometry elements added: " << appendCount;
    logging::DEBUG(ss.str());
    return;
  }

  logging::WARN(
    "ScenePart::addObj ignored because ScenePart is not triangle-backed");
}

std::vector<Vertex*>
ScenePart::getAllVertices() const
{
  mVertexScratch.clear();
  for (std::size_t i = 0; i < geometryCount(); ++i) {
    std::size_t const n = geometryDynamicVertexCount(i);
    if (n == 0) {
      std::shared_ptr<AABB> box = geometryAABB(i);
      if (box != nullptr) {
        Vertex v;
        v.pos = box->getCentroid();
        mVertexScratch.push_back(v);
      }
      continue;
    }
    for (std::size_t j = 0; j < n; ++j) {
      Vertex v;
      v.pos = geometryDynamicVertexPosition(i, j);
      v.normal = geometryDynamicVertexNormal(i, j);
      mVertexScratch.push_back(v);
    }
  }

  std::vector<Vertex*> allPos;
  allPos.reserve(mVertexScratch.size());
  for (Vertex& v : mVertexScratch) {
    allPos.push_back(&v);
  }
  return allPos;
}

void
ScenePart::smoothVertexNormals()
{
  if (TriangleScenePart* trianglePart =
        dynamic_cast<TriangleScenePart*>(this)) {
    if (trianglePart->vertices.n_rows == 0) {
      return;
    }

    using Key = std::array<double, 3>;
    std::map<Key, glm::dvec3> normalSum;

    auto loadPos = [&](std::size_t row, std::size_t vertex) {
      std::size_t const base = 3 * vertex;
      return glm::dvec3(trianglePart->vertices(row, base),
                        trianglePart->vertices(row, base + 1),
                        trianglePart->vertices(row, base + 2));
    };
    auto keyOf = [](glm::dvec3 const& p) { return Key{ p.x, p.y, p.z }; };

    // Accumulate face normals per shared vertex position.
    for (std::size_t row = 0; row < trianglePart->vertices.n_rows; ++row) {
      glm::dvec3 const v0 = loadPos(row, 0);
      glm::dvec3 const v1 = loadPos(row, 1);
      glm::dvec3 const v2 = loadPos(row, 2);
      glm::dvec3 n = glm::cross(v1 - v0, v2 - v0);
      double const len = glm::length(n);
      if (len == 0.0) {
        continue;
      }
      n /= len;
      normalSum[keyOf(v0)] += n;
      normalSum[keyOf(v1)] += n;
      normalSum[keyOf(v2)] += n;
    }

    trianglePart->normals.set_size(trianglePart->vertices.n_rows,
                                   trianglePart->vertices.n_cols);
    for (std::size_t row = 0; row < trianglePart->vertices.n_rows; ++row) {
      for (std::size_t vertex = 0; vertex < 3; ++vertex) {
        std::size_t const base = 3 * vertex;
        glm::dvec3 const p = loadPos(row, vertex);
        glm::dvec3 n = normalSum[keyOf(p)];
        double const len = glm::length(n);
        if (len != 0.0) {
          n /= len;
        }
        trianglePart->normals(row, base) = n.x;
        trianglePart->normals(row, base + 1) = n.y;
        trianglePart->normals(row, base + 2) = n.z;
      }
    }
  }
}

bool
ScenePart::splitSubparts(std::vector<std::shared_ptr<ScenePart>>* splitParts)
{
  size_t n = subpartLimit.size();
  if (n <= 1)
    return false; // There is no need to do splits

  // Prepare incremental ID
  int start = -1;
  try {
    start = boost::lexical_cast<int>(mId);
  } catch (boost::bad_lexical_cast& blcex) {
    std::stringstream ss;
    ss << "Could not update subpart ID from \"" << mId << "\".\n "
       << "Thus, splitting subparts is aborted";
    logging::WARN(ss.str());
    return false;
  }

  auto clampEnd = [](std::size_t begin, std::size_t end, std::size_t size) {
    if (begin >= size) {
      return begin;
    }
    return std::min(end, size);
  };
  auto sliceTriangle =
    [&](TriangleScenePart& part, std::size_t begin, std::size_t end) {
      std::size_t const last = clampEnd(begin, end, part.vertices.n_rows);
      if (begin >= last) {
        part.vertices.reset();
        part.normals.reset();
        part.materialIndex.reset();
        return;
      }
      part.vertices = part.vertices.rows(begin, last - 1);
      if (part.normals.n_rows > 0) {
        part.normals = part.normals.rows(begin, last - 1);
      }
      if (part.materialIndex.n_elem > 0) {
        part.materialIndex = part.materialIndex.subvec(begin, last - 1);
      }
    };
  auto sliceVoxel =
    [&](VoxelScenePart& part, std::size_t begin, std::size_t end) {
      std::size_t const last = clampEnd(begin, end, part.centers.n_rows);
      if (begin >= last) {
        part.centers.reset();
        part.halfSizes.reset();
        part.normals.reset();
        part.materialIndex.reset();
        return;
      }
      part.centers = part.centers.rows(begin, last - 1);
      part.halfSizes = part.halfSizes.rows(begin, last - 1);
      if (part.normals.n_rows > 0) {
        part.normals = part.normals.rows(begin, last - 1);
      }
      if (part.materialIndex.n_elem > 0) {
        part.materialIndex = part.materialIndex.subvec(begin, last - 1);
      }
    };
  auto sliceDetailedVoxel =
    [&](DetailedVoxelScenePart& part, std::size_t begin, std::size_t end) {
      sliceVoxel(part, begin, end);
      std::size_t const last = clampEnd(begin, end, part.intData.n_rows);
      if (begin >= last) {
        part.intData.reset();
        part.doubleData.reset();
        part.identifiers.reset();
        return;
      }
      if (part.intData.n_rows > 0) {
        part.intData = part.intData.rows(begin, last - 1);
      }
      if (part.doubleData.n_rows > 0) {
        part.doubleData = part.doubleData.rows(begin, last - 1);
      }
      if (part.identifiers.n_elem > 0) {
        part.identifiers = part.identifiers.subvec(begin, last - 1);
      }
    };
  auto cloneBulkRange = [&](std::size_t begin,
                            std::size_t end) -> std::shared_ptr<ScenePart> {
    if (DetailedVoxelScenePart* detailed =
          dynamic_cast<DetailedVoxelScenePart*>(this)) {
      std::shared_ptr<DetailedVoxelScenePart> out =
        std::make_shared<DetailedVoxelScenePart>(*detailed);
      sliceDetailedVoxel(*out, begin, end);
      return out;
    }
    if (VoxelScenePart* voxel = dynamic_cast<VoxelScenePart*>(this)) {
      std::shared_ptr<VoxelScenePart> out =
        std::make_shared<VoxelScenePart>(*voxel);
      sliceVoxel(*out, begin, end);
      return out;
    }
    if (TriangleScenePart* triangle = dynamic_cast<TriangleScenePart*>(this)) {
      std::shared_ptr<TriangleScenePart> out =
        std::make_shared<TriangleScenePart>(*triangle);
      sliceTriangle(*out, begin, end);
      return out;
    }
    return nullptr;
  };

  if (dynamic_cast<TriangleScenePart*>(this) != nullptr ||
      dynamic_cast<VoxelScenePart*>(this) != nullptr) {
    for (std::size_t i = 1; i < n; ++i) {
      std::shared_ptr<ScenePart> newPart =
        cloneBulkRange(subpartLimit[i - 1], subpartLimit[i]);
      if (newPart == nullptr) {
        break;
      }
      newPart->subpartLimit.clear();
      newPart->mId = std::to_string(start + i);
      if (splitParts != nullptr) {
        splitParts->push_back(newPart);
      }
    }

    if (DetailedVoxelScenePart* detailed =
          dynamic_cast<DetailedVoxelScenePart*>(this)) {
      sliceDetailedVoxel(*detailed, 0, subpartLimit[0]);
    } else if (VoxelScenePart* voxel = dynamic_cast<VoxelScenePart*>(this)) {
      sliceVoxel(*voxel, 0, subpartLimit[0]);
    } else if (TriangleScenePart* triangle =
                 dynamic_cast<TriangleScenePart*>(this)) {
      sliceTriangle(*triangle, 0, subpartLimit[0]);
    }
    subpartLimit.clear();
    mId = std::to_string(start);
    return true;
  }

  return false;
}

void
ScenePart::computeCentroid(bool const computeBound)
{
  double xmin = std::numeric_limits<double>::max();
  double ymin = std::numeric_limits<double>::max();
  double zmin = std::numeric_limits<double>::max();
  double xmax = std::numeric_limits<double>::lowest();
  double ymax = std::numeric_limits<double>::lowest();
  double zmax = std::numeric_limits<double>::lowest();
  bool found = false;

  for (std::size_t i = 0; i < geometryCount(); ++i) {
    std::size_t const n = geometryDynamicVertexCount(i);
    if (n > 0) {
      for (std::size_t j = 0; j < n; ++j) {
        glm::dvec3 const p = geometryDynamicVertexPosition(i, j);
        xmin = std::min(xmin, p.x);
        ymin = std::min(ymin, p.y);
        zmin = std::min(zmin, p.z);
        xmax = std::max(xmax, p.x);
        ymax = std::max(ymax, p.y);
        zmax = std::max(zmax, p.z);
        found = true;
      }
      continue;
    }

    std::shared_ptr<AABB> box = geometryAABB(i);
    if (box != nullptr) {
      glm::dvec3 const& mn = box->getMin();
      glm::dvec3 const& mx = box->getMax();
      xmin = std::min(xmin, mn.x);
      ymin = std::min(ymin, mn.y);
      zmin = std::min(zmin, mn.z);
      xmax = std::max(xmax, mx.x);
      ymax = std::max(ymax, mx.y);
      zmax = std::max(zmax, mx.z);
      found = true;
    }
  }

  if (!found) {
    centroid = arma::colvec({ 0.0, 0.0, 0.0 });
    if (computeBound) {
      bound = nullptr;
    }
    return;
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
ScenePart::computeTransformations(std::shared_ptr<ScenePart> sp,
                                  bool const holistic)
{
  if (sp == nullptr) {
    return;
  }
  // For all geometry elements, set owner and apply transformations.
  sp->bindGeometryOwners(sp);
  for (std::size_t i = 0; i < sp->geometryCount(); ++i) {
    sp->geometryRotate(i, sp->mRotation);
    if (holistic) {
      // Keep legacy semantics for holistic transformations.
      sp->geometryScale(i, sp->mScale);
    }
    sp->geometryScale(i, sp->mScale);
    sp->geometryTranslate(i, sp->mOrigin);
    sp->geometryUpdate(i);
  }
}

void
ScenePart::release()
{
  deleteGeometries();
  if (sorh != nullptr) {
    sorh->releaseBaselineGeometries();
    sorh->baseline = nullptr;
    sorh = nullptr;
  }
}

std::size_t
ScenePart::geometryCount() const
{
  return 0;
}

ScenePart::GeometryType
ScenePart::geometryTypeOf() const
{
  return geometryType;
}

std::shared_ptr<AABB>
ScenePart::computeBound()
{
  computeCentroid(true);
  return bound;
}

glm::dvec3
ScenePart::geometryCentroid(std::size_t) const
{
  return glm::dvec3(0.0);
}

double
ScenePart::geometryRayIntersectionDistance(std::size_t,
                                           glm::dvec3 const&,
                                           glm::dvec3 const&) const
{
  return -1.0;
}

std::vector<double>
ScenePart::geometryRayIntersection(std::size_t,
                                   glm::dvec3 const&,
                                   glm::dvec3 const&) const
{
  return {};
}

double
ScenePart::geometryIncidenceAngle(std::size_t,
                                  glm::dvec3 const&,
                                  glm::dvec3 const&,
                                  glm::dvec3 const&) const
{
  return 0.0;
}

std::size_t
ScenePart::geometryVertexCount(std::size_t) const
{
  return 0;
}

std::size_t
ScenePart::geometryDynamicVertexCount(std::size_t index) const
{
  return geometryVertexCount(index);
}

glm::dvec3
ScenePart::geometryDynamicVertexPosition(std::size_t, std::size_t) const
{
  return glm::dvec3(0.0);
}

glm::dvec3
ScenePart::geometryDynamicVertexNormal(std::size_t, std::size_t) const
{
  return glm::dvec3(0.0);
}

void
ScenePart::setGeometryDynamicVertexPosition(std::size_t,
                                            std::size_t,
                                            glm::dvec3 const&)
{
}

void
ScenePart::setGeometryDynamicVertexNormal(std::size_t,
                                          std::size_t,
                                          glm::dvec3 const&)
{
}

double
ScenePart::geometryGroundZOffset(std::size_t) const
{
  return 0.0;
}

bool
ScenePart::geometryCanComputeSigmaWithLadLut(std::size_t) const
{
  return false;
}

double
ScenePart::geometryComputeSigmaWithLadLut(std::size_t, glm::dvec3 const&) const
{
  return 0.0;
}

bool
ScenePart::geometryCanHandleIntersections(std::size_t) const
{
  return false;
}

std::shared_ptr<AABB>
ScenePart::geometryAABB(std::size_t index) const
{
  std::size_t const n = geometryVertexCount(index);
  if (n == 0) {
    return nullptr;
  }
  glm::dvec3 p = geometryDynamicVertexPosition(index, 0);
  double minX = p.x, minY = p.y, minZ = p.z;
  double maxX = p.x, maxY = p.y, maxZ = p.z;
  for (std::size_t i = 1; i < n; ++i) {
    p = geometryDynamicVertexPosition(index, i);
    minX = std::min(minX, p.x);
    minY = std::min(minY, p.y);
    minZ = std::min(minZ, p.z);
    maxX = std::max(maxX, p.x);
    maxY = std::max(maxY, p.y);
    maxZ = std::max(maxZ, p.z);
  }
  return std::make_shared<AABB>(glm::dvec3(minX, minY, minZ),
                                glm::dvec3(maxX, maxY, maxZ));
}

IntersectionHandlingResult
ScenePart::geometryOnRayIntersection(std::size_t,
                                     NoiseSource<double>&,
                                     glm::dvec3&,
                                     glm::dvec3 const&,
                                     glm::dvec3 const&,
                                     double) const
{
  return {};
}

std::shared_ptr<Material>
ScenePart::geometryMaterial(std::size_t) const
{
  return nullptr;
}

void
ScenePart::setGeometryMaterial(std::size_t, std::shared_ptr<Material>)
{
}

void
ScenePart::bindGeometryOwners(std::shared_ptr<ScenePart> const&)
{
}

void
ScenePart::bindUnownedGeometryOwners(std::shared_ptr<ScenePart> const&)
{
}

void
ScenePart::geometryRotate(std::size_t, Rotation&)
{
}

void
ScenePart::geometryScale(std::size_t, double)
{
}

void
ScenePart::geometryTranslate(std::size_t, glm::dvec3 const&)
{
}

void
ScenePart::geometryUpdate(std::size_t)
{
}

void
ScenePart::geometryOnFinishLoading(std::size_t, NoiseSource<double>&)
{
}

void
ScenePart::deleteGeometries()
{
  clearGeometryStorage();
}

void
ScenePart::clearGeometryStorage()
{
}

void
ScenePart::unbindGeometryOwners()
{
}
