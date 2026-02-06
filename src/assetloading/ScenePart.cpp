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
#include <util/logger/logging.hpp>

#include <glm/glm.hpp>

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
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
  this->triangles = sp.triangles;
  this->voxels = sp.voxels;
  this->detailed_voxels = sp.detailed_voxels;
  this->mPrimitives = std::vector<Primitive*>(0);
  Primitive* p;

  if (shallowPrimitives) {
    for (size_t i = 0; i < sp.mPrimitives.size(); ++i) {
      this->mPrimitives.push_back(sp.mPrimitives[i]);
    }
  } else {
    for (size_t i = 0; i < sp.mPrimitives.size(); ++i) {
      p = sp.mPrimitives[i]->clone();
      p->part = sp.mPrimitives[i]->part;
      this->mPrimitives.push_back(p);
    }
  }

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

  this->primitiveType = rhs.primitiveType;
  this->triangles = rhs.triangles;
  this->voxels = rhs.voxels;
  this->detailed_voxels = rhs.detailed_voxels;
  this->mPrimitives = std::vector<Primitive*>(0);
  Primitive* p;
  for (size_t i = 0; i < rhs.mPrimitives.size(); i++) {
    p = rhs.mPrimitives[i]->clone();
    p->part = rhs.mPrimitives[i]->part;
    this->mPrimitives.push_back(p);
  }

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
  size_t oldNumPrimitives = mPrimitives.size();

  Primitive* p;
  for (size_t i = 0; i < obj->primitives.size(); i++) {
    p = obj->primitives[i]->clone();
    mPrimitives.push_back(p);
  }

  ss << "# new primitives added: " << mPrimitives.size() - oldNumPrimitives;
  logging::DEBUG(ss.str());
}

std::vector<Vertex*>
ScenePart::getAllVertices() const
{
  std::vector<Vertex*> allPos;
  for (Primitive* p : mPrimitives) {
    for (size_t i = 0; i < p->getNumVertices(); i++) {
      allPos.push_back(p->getVertices() + i);
    }
  }
  return allPos;
}

void
ScenePart::smoothVertexNormals()
{
  Triangle* t;
  Vertex* v;
  std::map<Vertex*, std::shared_ptr<std::vector<Triangle*>>> vtmap;

  // Build map so for each vertex all triangles formed by it are known
  for (size_t i = 0; i < mPrimitives.size(); i++) {
    t = (Triangle*)mPrimitives[i];
    for (int j = 0; j <= 2; j++) {
      if (!vtmap.count(t->verts + j)) {
        std::shared_ptr<std::vector<Triangle*>> vec =
          std::make_shared<std::vector<Triangle*>>();
        vtmap.insert({ t->verts + j, vec });
      }
      vtmap[t->verts + j]->push_back(t);
    }
  }

  // Set the normal of each vertex as the mean of each triangle normal
  std::shared_ptr<std::vector<Triangle*>> vec;
  for (std::map<Vertex*, std::shared_ptr<std::vector<Triangle*>>>::iterator
         iter = vtmap.begin();
       iter != vtmap.end();
       iter++) {
    v = iter->first;
    vec = iter->second;
    v->normal[0] = 0.0;
    v->normal[1] = 0.0;
    v->normal[2] = 0.0;
    for (Triangle* t : *vec) {
      v->normal += t->getFaceNormal();
    }
    v->normal = glm::normalize(v->normal);
  }
}

bool
ScenePart::splitSubparts()
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
    for (size_t j = subpartLimit[i - 1]; j < subpartLimit[i]; ++j) {
      newPart->mPrimitives.push_back(mPrimitives[j]);
      mPrimitives[j]->part = newPart;
    }
    newPart->mId = std::to_string(start + i);
  }

  // Remove splitted primitives
  mPrimitives.erase(mPrimitives.begin() + subpartLimit[0], mPrimitives.end());
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
  return true;
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
ScenePart::buildBulkFromPrimitives()
{
  clearBulkData();
  if (mPrimitives.empty())
    return;

  if (primitiveType == PrimitiveType::TRIANGLE) {
    std::size_t const n = mPrimitives.size();
    triangles.vertices.reserve(3 * n);
    triangles.face_normal.reserve(n);
    triangles.e1.reserve(n);
    triangles.e2.reserve(n);
    triangles.v0.reserve(n);
    triangles.eps.reserve(n);
    triangles.aabb_min.reserve(n);
    triangles.aabb_max.reserve(n);
    triangles.materials.reserve(n);

    for (Primitive* primitive : mPrimitives) {
      Triangle* t = dynamic_cast<Triangle*>(primitive);
      if (t == nullptr) {
        logging::WARN(
          "ScenePart::buildBulkFromPrimitives found non-triangle primitive");
        clearBulkData();
        return;
      }

      Vertex* verts = t->getVertices();
      triangles.vertices.push_back(verts[0]);
      triangles.vertices.push_back(verts[1]);
      triangles.vertices.push_back(verts[2]);

      glm::dvec3 v0 = verts[0].pos;
      glm::dvec3 e1 = verts[1].pos - v0;
      glm::dvec3 e2 = verts[2].pos - v0;
      glm::dvec3 normal = glm::cross(e1, e2);
      double const normal_len = glm::length(normal);
      if (normal_len > 0.0)
        normal = normal / normal_len;

      triangles.v0.push_back(v0);
      triangles.e1.push_back(e1);
      triangles.e2.push_back(e2);
      triangles.face_normal.push_back(normal);
      triangles.eps.push_back(0.0000001);

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
      triangles.aabb_min.push_back(glm::dvec3(minX, minY, minZ));
      triangles.aabb_max.push_back(glm::dvec3(maxX, maxY, maxZ));

      triangles.materials.push_back(t->material);
    }
    return;
  }

  if (primitiveType == PrimitiveType::VOXEL) {
    std::size_t const n = mPrimitives.size();
    voxels.centers.reserve(n);
    voxels.half_size.reserve(n);
    voxels.num_points.reserve(n);
    voxels.r.reserve(n);
    voxels.g.reserve(n);
    voxels.b.reserve(n);
    voxels.color.reserve(n);
    voxels.aabb_min.reserve(n);
    voxels.aabb_max.reserve(n);
    voxels.materials.reserve(n);

    detailed_voxels.present.reserve(n);
    detailed_voxels.int_values.reserve(n);
    detailed_voxels.double_values.reserve(n);
    detailed_voxels.max_pad.reserve(n);

    for (Primitive* primitive : mPrimitives) {
      Voxel* v = dynamic_cast<Voxel*>(primitive);
      if (v == nullptr) {
        logging::WARN(
          "ScenePart::buildBulkFromPrimitives found non-voxel primitive");
        clearBulkData();
        return;
      }

      voxels.centers.push_back(v->v);
      voxels.half_size.push_back(v->halfSize);
      voxels.num_points.push_back(v->numPoints);
      voxels.r.push_back(v->r);
      voxels.g.push_back(v->g);
      voxels.b.push_back(v->b);
      voxels.color.push_back(v->color);
      voxels.materials.push_back(v->material);

      glm::dvec3 hs = glm::dvec3(v->halfSize, v->halfSize, v->halfSize);
      voxels.aabb_min.push_back(v->v.pos - hs);
      voxels.aabb_max.push_back(v->v.pos + hs);

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
ScenePart::computeTransformations(std::shared_ptr<ScenePart> sp,
                                  bool const holistic)
{
  // For all primitives, set reference to their scene part and transform:
  for (Primitive* p : sp->mPrimitives) {
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
    delete p;
  }
  mPrimitives.clear();
  clearBulkData();
  if (sorh != nullptr) {
    for (Primitive* p : sorh->getBaselinePrimitives()) {
      delete p;
    }
    sorh->baseline = nullptr;
    sorh = nullptr;
  }
}
