// #include <iostream>
#include "logging.hpp"

#include <KDTreeRaycaster.h>
#include <Scene.h>
#include <SerialIO.h>
#include <TimeWatcher.h>
#include <UniformNoiseSource.h>
#include <surfaceinspector/maths/Plane.hpp>
#include <surfaceinspector/maths/PlaneFitter.hpp>

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <limits>
#include <unordered_set>

#if DATA_ANALYTICS >= 2
#include <dataanalytics/HDA_GlobalVars.h>
using helios::analytics::HDA_GV;
#endif

using SurfaceInspector::maths::Plane;
using SurfaceInspector::maths::PlaneFitter;

namespace {
std::shared_ptr<AABB>
computeSceneBoundFromParts(std::vector<std::shared_ptr<ScenePart>> const& parts)
{
  double minX = std::numeric_limits<double>::max();
  double minY = std::numeric_limits<double>::max();
  double minZ = std::numeric_limits<double>::max();
  double maxX = std::numeric_limits<double>::lowest();
  double maxY = std::numeric_limits<double>::lowest();
  double maxZ = std::numeric_limits<double>::lowest();
  bool found = false;
  for (std::shared_ptr<ScenePart> const& part : parts) {
    if (part == nullptr || part->geometryCount() == 0) {
      continue;
    }
    std::shared_ptr<AABB> const partBound = part->computeBound();
    if (partBound == nullptr) {
      continue;
    }
    glm::dvec3 const& mn = partBound->getMin();
    glm::dvec3 const& mx = partBound->getMax();
    minX = std::min(minX, mn.x);
    minY = std::min(minY, mn.y);
    minZ = std::min(minZ, mn.z);
    maxX = std::max(maxX, mx.x);
    maxY = std::max(maxY, mx.y);
    maxZ = std::max(maxZ, mx.z);
    found = true;
  }
  if (!found) {
    return nullptr;
  }
  return std::make_shared<AABB>(glm::dvec3(minX, minY, minZ),
                                glm::dvec3(maxX, maxY, maxZ));
}

std::size_t
countSceneVerticesFromParts(
  std::vector<std::shared_ptr<ScenePart>> const& parts)
{
  std::size_t numVertices = 0;
  for (std::shared_ptr<ScenePart> const& part : parts) {
    if (part == nullptr) {
      continue;
    }
    for (std::size_t i = 0; i < part->geometryCount(); ++i) {
      numVertices += part->geometryVertexCount(i);
    }
  }
  return numVertices;
}

std::vector<glm::dvec3>
collectPartDynamicVertices(ScenePart const& part)
{
  std::vector<glm::dvec3> vertices;
  for (std::size_t i = 0; i < part.geometryCount(); ++i) {
    std::size_t const n = part.geometryDynamicVertexCount(i);
    for (std::size_t j = 0; j < n; ++j) {
      vertices.push_back(part.geometryDynamicVertexPosition(i, j));
    }
  }
  return vertices;
}
}

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
Scene::Scene(Scene& s)
{
  if (s.bbox == nullptr)
    this->bbox = nullptr;
  else
    this->bbox = std::make_shared<AABB>(*s.bbox);
  if (s.bbox_crs == nullptr)
    this->bbox_crs = nullptr;
  else
    this->bbox_crs = std::make_shared<AABB>(*s.bbox_crs);
  for (std::shared_ptr<ScenePart> const& part : s.parts) {
    if (part == nullptr) {
      continue;
    }
    std::shared_ptr<ScenePart> cloned = part->clone(false);
    cloned->bindGeometryOwners(cloned);
    parts.push_back(cloned);
  }

  this->kdgf = s.kdgf;
  registerParts();
  rebuildGeometryRefs();
  if (s.parts.empty()) {
    kdgrove = nullptr;
  } else {
    buildKDGrove(true);
  }
}

// ***  M E T H O D S  *** //
// *********************** //
bool
Scene::finalizeLoading(bool const safe)
{
  registerParts();
  rebuildGeometryRefs();
  if (geometryRefs.empty())
    return false;

  // #####   UPDATE SCENE PART GEOMETRY ON FINISH LOADING   #####
  UniformNoiseSource<double> uns(*DEFAULT_RG, -1, 1);
  for (GeometryRef const& ref : geometryRefs) {
    if (ref.isValid()) {
      ref.part->geometryOnFinishLoading(ref.index, uns);
    }
  }

  // Report number of primitives in the scene
  std::ostringstream s;
  s << "Total # of primitives in scene: " << geometryRefs.size() << "\n";
  logging::DEBUG(s.str());

  // Compute the number of vertices in the scene
  std::size_t const numVertices = countSceneVerticesFromParts(parts);

  // Translate to ground those flagged as forceOnGround
  doForceOnGround();

  // Store original bounding box (CRS coordinates):
  this->bbox_crs = computeSceneBoundFromParts(parts);
  if (this->bbox_crs == nullptr) {
    return false;
  }
  glm::dvec3 const diff = this->bbox_crs->getCentroid();
  std::stringstream ss;
  ss << "CRS bounding box (by vertices): " << this->bbox_crs->toString()
     << "\nShift: " << glm::to_string(diff)
     << "\n# vertices to translate: " << numVertices;
  logging::INFO(ss.str());
  ss.str("");

  // Translate each part geometry to scene-local coordinates.
  for (std::shared_ptr<ScenePart>& part : parts) {
    if (part == nullptr) {
      continue;
    }
    for (std::size_t i = 0; i < part->geometryCount(); ++i) {
      part->geometryTranslate(i, -diff);
      part->geometryUpdate(i);
    }
  }

  // Get new bounding box of translated scene:
  this->bbox = computeSceneBoundFromParts(parts);

  if (this->bbox != nullptr) {
    ss << "Actual bounding box (by vertices): " << this->bbox->toString();
    logging::INFO(ss.str());
    ss.str("");
  }

  // ################ END Shift primitives to originWaypoint ##################

  // Compute each part centroid wrt to scene
  for (std::shared_ptr<ScenePart>& part : parts)
    part->computeCentroid();

  // Build KDGrove
  if (kdgf != nullptr)
    buildKDGroveWithLog(safe);

  return true;
}

void
Scene::registerParts()
{
  // Keep pre-existing unique parts.
  std::unordered_set<ScenePart*> seen;
  std::vector<std::shared_ptr<ScenePart>> registered;
  registered.reserve(parts.size());

  for (std::shared_ptr<ScenePart> const& part : parts) {
    if (part == nullptr) {
      continue;
    }
    if (seen.insert(part.get()).second) {
      registered.push_back(part);
    }
  }
  parts.swap(registered);
}

void
Scene::clearGeometryRefs()
{
  geometryRefs.clear();
}

void
Scene::rebuildGeometryRefs()
{
  geometryRefs.clear();
  std::size_t total = 0;
  for (std::shared_ptr<ScenePart> const& part : parts) {
    if (part != nullptr) {
      total += part->geometryCount();
    }
  }
  geometryRefs.reserve(total);
  for (std::shared_ptr<ScenePart> const& part : parts) {
    if (part == nullptr) {
      continue;
    }
    for (std::size_t i = 0; i < part->geometryCount(); ++i) {
      geometryRefs.push_back({ part, i });
    }
  }
}

std::shared_ptr<AABB>
Scene::getAABB()
{
  return this->bbox;
}

glm::dvec3
Scene::getGroundPointAt(glm::dvec3 point)
{

  glm::dvec3 origin = glm::dvec3(point.x, point.y, bbox->getMin()[2] - 0.1);
  glm::dvec3 dir = glm::dvec3(0, 0, 1);

  std::shared_ptr<RaySceneIntersection> intersect =
    getIntersection(origin, dir, true);

  if (intersect == nullptr) {
    std::stringstream ss;
    ss << "getGroundPointAt(" << point.x << "," << point.y << "," << point.z
       << ") : intersect is NULL";
    ss << "\n\torigin = (" << origin.x << ", " << origin.y << ", " << origin.z
       << ");\n\tdir = (" << dir.x << ", " << dir.y << ", " << dir.z << ");"
       << std::endl;
    logging::DEBUG(ss.str());
    return {};
  }

  intersect->point.z += intersect->geometryRef.groundZOffset();
  return intersect->point;
}

std::shared_ptr<RaySceneIntersection>
Scene::getIntersection(glm::dvec3 const& rayOrigin,
                       glm::dvec3 const& rayDir,
                       bool const groundOnly) const
{
  std::vector<double> tMinMax = bbox->getRayIntersection(rayOrigin, rayDir);
  return getIntersection(tMinMax, rayOrigin, rayDir, groundOnly);
}

std::shared_ptr<RaySceneIntersection>
Scene::getIntersection(std::vector<double> const& tMinMax,
                       glm::dvec3 const& rayOrigin,
                       glm::dvec3 const& rayDir,
                       bool const groundOnly) const
{
  if (tMinMax.empty()) {
    logging::DEBUG("tMinMax is empty");
#if DATA_ANALYTICS >= 2
    HDA_GV.incrementNonIntersectiveSubraysDueToNullTimeCount();
#endif
    return nullptr;
  }
  std::shared_ptr<RaySceneIntersection> intersection =
    std::shared_ptr<RaySceneIntersection>(
      raycaster->search(rayOrigin, rayDir, tMinMax[0], tMinMax[1], groundOnly));
  return intersection;
}

std::map<double, GeometryRef>
Scene::getIntersections(glm::dvec3& rayOrigin,
                        glm::dvec3& rayDir,
                        bool const groundOnly)
{

  std::vector<double> tMinMax = bbox->getRayIntersection(rayOrigin, rayDir);
  if (tMinMax.empty()) {
    logging::DEBUG("tMinMax is empty");
    return {};
  }

  return raycaster->searchAll(
    rayOrigin, rayDir, tMinMax[0], tMinMax[1], groundOnly);
}

glm::dvec3
Scene::getShift()
{
  return this->bbox_crs->getCentroid();
}

std::vector<Vertex*>
Scene::getAllVertices()
{
  mVertexScratch.clear();
  for (std::shared_ptr<ScenePart> const& part : parts) {
    if (part == nullptr) {
      continue;
    }
    for (std::size_t i = 0; i < part->geometryCount(); ++i) {
      std::size_t const n = part->geometryDynamicVertexCount(i);
      if (n > 0) {
        for (std::size_t j = 0; j < n; ++j) {
          Vertex v;
          v.pos = part->geometryDynamicVertexPosition(i, j);
          v.normal = part->geometryDynamicVertexNormal(i, j);
          mVertexScratch.push_back(v);
        }
        continue;
      }
      std::shared_ptr<AABB> const box = part->geometryAABB(i);
      if (box != nullptr) {
        Vertex v;
        v.pos = box->getCentroid();
        mVertexScratch.push_back(v);
      }
    }
  }

  std::vector<Vertex*> out;
  out.reserve(mVertexScratch.size());
  for (Vertex& v : mVertexScratch) {
    out.push_back(&v);
  }
  return out;
}

void
Scene::doForceOnGround()
{
  // 1. Find min and max vertices of ground scene parts
  std::vector<std::size_t> I; // Indices of ground parts
  std::vector<std::unique_ptr<SurfaceInspector::maths::Plane<double>>>
    planes;                           // Ground best fitting planes
  std::size_t const m = parts.size(); // How many parts there are in the scene
  for (std::size_t i = 0; i < m; ++i) {
    std::shared_ptr<ScenePart> part = parts[i];
    if (part == nullptr || part->isNull())
      continue;
    if (part->geometryCount() == 0)
      continue;
    std::shared_ptr<Material> partMaterial = part->geometryMaterial(0);
    if (partMaterial == nullptr || !partMaterial->isGround)
      continue;
    I.push_back(i);              // Store index of found ground part
    planes.push_back(nullptr);   // Null placeholder for best fitting plane
    part->computeCentroid(true); // True implies also store boundaries
  }
  if (I.empty()) { // Check there is at least one ground part
    std::stringstream ss;
    ss << "Scene::doForceGround could not compute nothing because there "
       << "was no ground scene part available";
    logging::WARN(ss.str());
  }

  // Compute remaining algorithm steps for each on ground scene part
  for (std::shared_ptr<ScenePart>& part : parts) {
    if (part == nullptr || part->geometryCount() == 0) {
      continue;
    }
    if (part->forceOnGround == 0) { // Ignore not on ground scene parts
      std::stringstream ss;
      ss << "Scene::doForceOnGround skipped part \"" << part->mId << "\"\n"
         << "Its forceOnGround attribute was 0";
      logging::DEBUG(ss.str());
      continue;
    } else { // Report search depth (forceOnGround) as debug info
      std::stringstream ss;
      ss << "Scene::doForceOnGround computing part \"" << part->mId << "\"\n"
         << "Search depth is " << part->forceOnGround;
      logging::DEBUG(ss.str());
    }
    // 2. Find minimum z vertex and pick first ground reference
    std::vector<glm::dvec3> const vertices = collectPartDynamicVertices(*part);
    if (vertices.empty()) {
      continue;
    }
    glm::dvec3 minzv = vertices[0]; // First vertex as minz candidate
    std::size_t const n = vertices.size();
    for (std::size_t i = 1; i < n; ++i) { // Find best minz candidate
      glm::dvec3 const& vertex = vertices[i];
      if (vertex.z < minzv.z)
        minzv = vertex;
    }
    std::size_t groundLocalIndex;
    std::shared_ptr<ScenePart> groundPart = nullptr; // Ground scene part
    for (std::size_t j = 0; j < I.size(); ++j) {
      std::size_t const i = I[j]; // Ground index i
      std::shared_ptr<ScenePart> groundCandidate = parts[i];
      if ( // Ground candidate is valid if minz vertex lies inside in R2
        minzv.x >= groundCandidate->bound->getMin().x &&
        minzv.x <= groundCandidate->bound->getMax().x &&
        minzv.y >= groundCandidate->bound->getMin().y &&
        minzv.y <= groundCandidate->bound->getMax().y) {
        groundPart = groundCandidate;
        groundLocalIndex = j;
        break;
      }
    }
    if (groundPart == nullptr) {
      std::stringstream ss;
      ss << "Scene::doForceOnGround could not place part \"" << part->mId
         << "\" on ground.\n"
         << "No valid ground candidate was found";
      logging::WARN(ss.str());
      continue;
    }
    // 3. Find ground reference best fitting plane
    if (planes[groundLocalIndex] == nullptr) { // Estimate plane if needed
      std::vector<glm::dvec3> const groundVertices =
        collectPartDynamicVertices(*groundPart);
      if (groundVertices.empty()) {
        continue;
      }
      std::size_t const ngv = groundVertices.size();
      std::size_t const ngv2 = 2 * ngv;
      arma::mat groundVerticesMatrix(ngv, 3);
      for (std::size_t i = 0; i < ngv; ++i) {
        glm::dvec3 const& vert = groundVertices[i];
        groundVerticesMatrix[i] = vert.x;
        groundVerticesMatrix[ngv + i] = vert.y;
        groundVerticesMatrix[ngv2 + i] = vert.z;
      }
      planes[groundLocalIndex] =
        std::unique_ptr<SurfaceInspector::maths::Plane<double>>(
          new SurfaceInspector::maths::Plane<double>(
            SurfaceInspector::maths::PlaneFitter::bestFittingPlaneSVD<double>(
              groundVerticesMatrix)));
    }
    // 4. Compute the vertical projection of min vertex on ground plane
    std::vector<double> const& o = planes[groundLocalIndex]->centroid;
    std::vector<double> const& v = planes[groundLocalIndex]->orthonormal;
    glm::dvec3 q =
      findForceOnGroundQ(part->forceOnGround, minzv, vertices, o, v);
    double zDelta = q.z - (v[0] * o[0] + v[1] * o[1] + v[2] * o[2] -
                           v[0] * q.x - v[1] * q.y) /
                            v[2];
    // 5. Do vertical translation for all vertex of onGround part
    for (std::size_t i = 0; i < part->geometryCount(); ++i) {
      std::size_t const nv = part->geometryDynamicVertexCount(i);
      for (std::size_t j = 0; j < nv; ++j) {
        glm::dvec3 vertex = part->geometryDynamicVertexPosition(i, j);
        vertex.z -= zDelta;
        part->setGeometryDynamicVertexPosition(i, j, vertex);
      }
      part->geometryUpdate(i);
    }
  }
}

glm::dvec3
Scene::findForceOnGroundQ(int const searchDepth,
                          glm::dvec3 const minzv,
                          std::vector<glm::dvec3> const& vertices,
                          std::vector<double> const& o,
                          std::vector<double> const& v)
{
  if (vertices.empty()) {
    return minzv;
  }
  if (searchDepth == -1 || searchDepth > 1) { // Iterative search process for q
    // Compute loop configuration
    std::size_t const maxIters =
      (searchDepth == -1) ? vertices.size()
                          : std::min<std::size_t>(searchDepth, vertices.size());
    std::size_t const stepSize =
      (searchDepth == -1)
        ? 1
        : std::max<std::size_t>(1, vertices.size() / maxIters);
    // Compute the iterative search itself : argmin zDelta
    double const dot = v[0] * o[0] + v[1] * o[1] + v[2] * o[2];
    glm::dvec3 qBest = vertices[0];
    double zDeltaBest =
      qBest.z - (dot - v[0] * qBest.x - v[1] * qBest.y) / v[2];
    for (std::size_t i = stepSize; i < vertices.size(); i += stepSize) {
      glm::dvec3 const& q = vertices[i];
      double const zDelta = q.z - (dot - v[0] * q.x - v[1] * q.y) / v[2];
      if (zDelta < zDeltaBest) {
        zDeltaBest = zDelta;
        qBest = q;
      }
    }
    return qBest;
  }
  // By default searchDepth 1 is assumed, so q=q_*
  return minzv;
}

void
Scene::buildKDGrove(bool const safe)
{
  kdgrove = kdgf->makeFromSceneParts(parts, // Scene parts
                                     true,  // Merge non moving
                                     safe,  // Safe
                                     true,  // Compute KDGrove stats
                                     true,  // Report KDGrove stats
                                     true,  // Compute KDTree stats
                                     true   // Report KDTree stats
  );
  raycaster = std::make_shared<KDGroveRaycaster>(kdgrove);
}

void
Scene::buildKDGroveWithLog(bool const safe)
{
  logging::INFO("Building KD-Grove... ");
  TimeWatcher kdgTw;
  kdgTw.start();
  buildKDGrove(safe);
  kdgTw.stop();
  std::stringstream ss;
  ss << "KDG built in " << kdgTw.getElapsedDecimalSeconds() << "s";
  logging::TIME(ss.str());
}

void
Scene::shutdown()
{
  kdgf = nullptr;
  kdgrove = nullptr;
  bbox = nullptr;
  bbox_crs = nullptr;
  raycaster = nullptr;
  // Release scene parts and primitives
  for (std::shared_ptr<ScenePart> part : parts) {
    part->release();
  }
  parts.clear();
  clearGeometryRefs();
}

std::vector<std::shared_ptr<ScenePart>>
Scene::getSwapOnRepeatObjects()
{
  std::vector<std::shared_ptr<ScenePart>> sorObjs;
  for (std::shared_ptr<ScenePart> sp : parts) {
    if (sp->getSwapOnRepeatHandler() != nullptr)
      sorObjs.push_back(sp);
  }
  return sorObjs;
}

// ***  READ/WRITE  *** //
// ******************** //
void
Scene::writeObject(string path)
{
  stringstream ss;
  ss << "Writing scene object to " << path << " ...";
  logging::INFO(ss.str());
  SerialIO::getInstance()->write<Scene>(path, this);
}

Scene*
Scene::readObject(string path)
{
  std::stringstream ss;
  ss << "Reading scene object from " << path << " ...";
  logging::INFO(ss.str());
  return SerialIO::getInstance()->read<Scene>(path);
}
