#include <scene/primitives/PrimitiveViews.h>

#include <DetailedVoxel.h>
#include <Triangle.h>
#include <Voxel.h>

#include <MathConstants.h>
#include <cmath>

// *** TriangleView *** //
Primitive*
TriangleView::clone()
{
  if (owner == nullptr)
    return nullptr;
  Vertex* verts = getVertices();
  if (verts == nullptr)
    return nullptr;
  Triangle* t = new Triangle(verts[0], verts[1], verts[2]);
  t->material = material;
  return t;
}

AABB*
TriangleView::getAABB()
{
  updateAabbCache();
  return &aabb_cache;
}

glm::dvec3
TriangleView::getCentroid()
{
  Vertex* verts = getVertices();
  if (verts == nullptr)
    return glm::dvec3(0.0, 0.0, 0.0);
  return (verts[0].pos + verts[1].pos + verts[2].pos) / 3.0;
}

double
TriangleView::getIncidenceAngle_rad(const glm::dvec3& rayOrigin,
                                    const glm::dvec3& rayDir,
                                    const glm::dvec3& intersectionPoint)
{
  glm::dvec3 const fn = getFaceNormal();
  double const angle = glm::angle(fn, rayDir);
  return (angle > PI_HALF) ? M_PI - angle : angle;
}

std::vector<double>
TriangleView::getRayIntersection(const glm::dvec3& rayOrigin,
                                 const glm::dvec3& rayDir)
{
  if (owner == nullptr)
    return std::vector<double>{ -1 };

  glm::dvec3 e1, e2, v0;
  double eps = 0.0000001;
  if (index < owner->triangles.e1.size())
    e1 = owner->triangles.e1[index];
  if (index < owner->triangles.e2.size())
    e2 = owner->triangles.e2[index];
  if (index < owner->triangles.v0.size())
    v0 = owner->triangles.v0[index];
  if (index < owner->triangles.eps.size())
    eps = owner->triangles.eps[index];
  if (owner->triangles.e1.size() <= index ||
      owner->triangles.e2.size() <= index ||
      owner->triangles.v0.size() <= index) {
    Vertex* verts = getVertices();
    if (verts == nullptr)
      return std::vector<double>{ -1 };
    v0 = verts[0].pos;
    e1 = verts[1].pos - v0;
    e2 = verts[2].pos - v0;
  }

  glm::dvec3 const h = glm::cross(rayDir, e2);
  double const a = e1.x * h.x + e1.y * h.y + e1.z * h.z;

  if (a > -eps && a < eps)
    return std::vector<double>{ -1 };

  double const f = 1.0 / a;
  glm::dvec3 const s = rayOrigin - v0;
  double const u = f * (s.x * h.x + s.y * h.y + s.z * h.z);

  if (u < 0.0 || u > 1.0)
    return std::vector<double>{ -1 };

  glm::dvec3 const q = glm::cross(s, e1);
  double const v = f * (rayDir.x * q.x + rayDir.y * q.y + rayDir.z * q.z);

  if (v < 0.0 || u + v > 1.0)
    return std::vector<double>{ -1 };

  double const t = f * (e2.x * q.x + e2.y * q.y + e2.z * q.z);

  if (t > eps)
    return std::vector<double>{ t };
  return std::vector<double>{ -1 };
}

double
TriangleView::getRayIntersectionDistance(const glm::dvec3& rayOrigin,
                                         const glm::dvec3& rayDir)
{
  if (owner == nullptr)
    return -1.0;

  glm::dvec3 e1, e2, v0;
  double eps = 0.0000001;
  if (index < owner->triangles.e1.size())
    e1 = owner->triangles.e1[index];
  if (index < owner->triangles.e2.size())
    e2 = owner->triangles.e2[index];
  if (index < owner->triangles.v0.size())
    v0 = owner->triangles.v0[index];
  if (index < owner->triangles.eps.size())
    eps = owner->triangles.eps[index];
  if (owner->triangles.e1.size() <= index ||
      owner->triangles.e2.size() <= index ||
      owner->triangles.v0.size() <= index) {
    Vertex* verts = getVertices();
    if (verts == nullptr)
      return -1.0;
    v0 = verts[0].pos;
    e1 = verts[1].pos - v0;
    e2 = verts[2].pos - v0;
  }

  const double hx = rayDir.y * e2.z - rayDir.z * e2.y;
  const double hy = rayDir.z * e2.x - rayDir.x * e2.z;
  const double hz = rayDir.x * e2.y - rayDir.y * e2.x;

  const double a = hx * e1.x + hy * e1.y + hz * e1.z;
  if (a > -eps && a < eps)
    return -1.0;

  const double sx = rayOrigin.x - v0.x;
  const double sy = rayOrigin.y - v0.y;
  const double sz = rayOrigin.z - v0.z;

  const double u = (sx * hx + sy * hy + sz * hz) / a;
  if (u < 0.0 || u > 1.0)
    return -1.0;

  const double qx = sy * e1.z - sz * e1.y;
  const double qy = sz * e1.x - sx * e1.z;
  const double qz = sx * e1.y - sy * e1.x;

  const double v = (rayDir.x * qx + rayDir.y * qy + rayDir.z * qz) / a;
  if (v < 0.0 || (u + v) > 1.0)
    return -1.0;

  const double t = (e2.x * qx + e2.y * qy + e2.z * qz) / a;
  if (t > eps)
    return t;
  return -1.0;
}

Vertex*
TriangleView::getVertices()
{
  if (owner == nullptr)
    return nullptr;
  std::size_t const base = 3 * index;
  if (base + 2 >= owner->triangles.vertices.size())
    return nullptr;
  return &owner->triangles.vertices[base];
}

void
TriangleView::update()
{
  if (owner == nullptr)
    return;
  Vertex* verts = getVertices();
  if (verts == nullptr)
    return;

  glm::dvec3 v0 = verts[0].pos;
  glm::dvec3 e1 = verts[1].pos - v0;
  glm::dvec3 e2 = verts[2].pos - v0;
  glm::dvec3 normal = glm::cross(e1, e2);
  double const len = glm::length(normal);
  if (len > 0.0)
    normal = normal / len;

  owner->triangles.v0[index] = v0;
  owner->triangles.e1[index] = e1;
  owner->triangles.e2[index] = e2;
  owner->triangles.face_normal[index] = normal;
  if (index >= owner->triangles.eps.size())
    owner->triangles.eps.resize(index + 1, 0.0000001);

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

  if (index >= owner->triangles.aabb_min.size()) {
    owner->triangles.aabb_min.resize(index + 1);
    owner->triangles.aabb_max.resize(index + 1);
  }
  owner->triangles.aabb_min[index] = glm::dvec3(minX, minY, minZ);
  owner->triangles.aabb_max[index] = glm::dvec3(maxX, maxY, maxZ);
}

glm::dvec3
TriangleView::getFaceNormal() const
{
  if (owner == nullptr || index >= owner->triangles.face_normal.size())
    return glm::dvec3(0.0, 0.0, 0.0);
  return owner->triangles.face_normal[index];
}

void
TriangleView::setAllVertexColors(Color4f color)
{
  Vertex* verts = getVertices();
  if (verts == nullptr)
    return;
  verts[0].color = color;
  verts[1].color = color;
  verts[2].color = color;
}

void
TriangleView::setAllVertexNormalsFromFace()
{
  glm::dvec3 fn = getFaceNormal();
  Vertex* verts = getVertices();
  if (verts == nullptr)
    return;
  verts[0].normal = fn;
  verts[1].normal = fn;
  verts[2].normal = fn;
}

double
TriangleView::calcArea2D()
{
  Vertex* verts = getVertices();
  if (verts == nullptr)
    return 0.0;
  double det = verts[0].getX() * (verts[1].getY() - verts[2].getY()) +
               verts[1].getX() * (verts[2].getY() - verts[0].getY()) +
               verts[2].getX() * (verts[0].getY() - verts[1].getY());
  return 0.5 * std::fabs(det);
}

double
TriangleView::calcArea3D()
{
  Vertex* verts = getVertices();
  if (verts == nullptr)
    return 0.0;
  glm::dvec3 ab = glm::dvec3(verts[1].getX() - verts[0].getX(),
                             verts[1].getY() - verts[0].getY(),
                             verts[1].getZ() - verts[0].getZ());
  glm::dvec3 ac = glm::dvec3(verts[2].getX() - verts[0].getX(),
                             verts[2].getY() - verts[0].getY(),
                             verts[2].getZ() - verts[0].getZ());
  double cross = glm::length(glm::cross(ab, ac));
  return 0.5 * cross;
}

void
TriangleView::updateAabbCache() const
{
  if (owner == nullptr || index >= owner->triangles.aabb_min.size())
    return;
  aabb_cache =
    AABB(owner->triangles.aabb_min[index], owner->triangles.aabb_max[index]);
}

// *** VoxelView *** //
Primitive*
VoxelView::clone()
{
  if (owner == nullptr)
    return nullptr;
  if (index >= owner->voxels.centers.size())
    return nullptr;

  Voxel* vox = new Voxel(owner->voxels.centers[index].pos.x,
                         owner->voxels.centers[index].pos.y,
                         owner->voxels.centers[index].pos.z,
                         owner->voxels.half_size[index]);
  vox->v = owner->voxels.centers[index];
  if (index < owner->voxels.num_points.size())
    vox->numPoints = owner->voxels.num_points[index];
  if (index < owner->voxels.r.size())
    vox->r = owner->voxels.r[index];
  if (index < owner->voxels.g.size())
    vox->g = owner->voxels.g[index];
  if (index < owner->voxels.b.size())
    vox->b = owner->voxels.b[index];
  if (index < owner->voxels.color.size())
    vox->color = owner->voxels.color[index];
  if (index < owner->voxels.materials.size())
    vox->material = owner->voxels.materials[index];
  vox->update();
  return vox;
}

AABB*
VoxelView::getAABB()
{
  updateAabbCache();
  return &aabb_cache;
}

glm::dvec3
VoxelView::getCentroid()
{
  Vertex* v = getVertices();
  if (v == nullptr)
    return glm::dvec3(0.0, 0.0, 0.0);
  return v->pos;
}

double
VoxelView::getIncidenceAngle_rad(const glm::dvec3& rayOrigin,
                                 const glm::dvec3& rayDir,
                                 const glm::dvec3& intersectionPoint)
{
  if (!hasNormal()) {
    return getIncidenceAngleClosestFace_rad(
      rayOrigin, rayDir, intersectionPoint);
  }
  Vertex* v = getVertices();
  double const angle = glm::angle(v->normal, rayDir);
  return (angle > PI_HALF) ? M_PI - angle : angle;
}

std::vector<double>
VoxelView::getRayIntersection(const glm::dvec3& rayOrigin,
                              const glm::dvec3& rayDir)
{
  return getAABB()->getRayIntersection(rayOrigin, rayDir);
}

double
VoxelView::getRayIntersectionDistance(const glm::dvec3& rayOrigin,
                                      const glm::dvec3& rayDir)
{
  return getAABB()->getRayIntersectionDistance(rayOrigin, rayDir);
}

Vertex*
VoxelView::getVertices()
{
  if (owner == nullptr)
    return nullptr;
  if (index >= owner->voxels.centers.size())
    return nullptr;
  return &owner->voxels.centers[index];
}

Vertex*
VoxelView::getFullVertices()
{
  updateAabbCache();
  return aabb_cache.getVertices();
}

double
VoxelView::getGroundZOffset()
{
  if (owner == nullptr || index >= owner->voxels.half_size.size())
    return 0.0;
  return owner->voxels.half_size[index] * 2.0;
}

void
VoxelView::update()
{
  if (owner == nullptr || index >= owner->voxels.centers.size())
    return;
  double halfSize = owner->voxels.half_size[index];
  glm::dvec3 hs(halfSize, halfSize, halfSize);
  owner->voxels.aabb_min[index] = owner->voxels.centers[index].pos - hs;
  owner->voxels.aabb_max[index] = owner->voxels.centers[index].pos + hs;
}

bool
VoxelView::hasNormal() const
{
  Vertex* v = const_cast<VoxelView*>(this)->getVertices();
  if (v == nullptr)
    return false;
  return v->normal[0] != 0.0 || v->normal[1] != 0.0 || v->normal[2] != 0.0;
}

double
VoxelView::getIncidenceAngleClosestFace_rad(const glm::dvec3& rayOrigin,
                                            const glm::dvec3& rayDir,
                                            const glm::dvec3& intersectionPoint)
{
  if (owner == nullptr || index >= owner->voxels.half_size.size())
    return 0.0;
  Vertex* v = getVertices();
  if (v == nullptr)
    return 0.0;

  double const halfSize = owner->voxels.half_size[index];
  glm::dvec3 normal(1, 0, 0);
  glm::dvec3 fc = v->pos + glm::dvec3(halfSize, 0, 0);
  double minDist = glm::distance(fc, intersectionPoint);
  fc = v->pos + glm::dvec3(-halfSize, 0, 0);
  double dist = glm::distance(fc, intersectionPoint);
  if (dist < minDist) {
    minDist = dist;
    normal = glm::dvec3(-1, 0, 0);
  }
  fc = v->pos + glm::dvec3(0, halfSize, 0);
  dist = glm::distance(fc, intersectionPoint);
  if (dist < minDist) {
    minDist = dist;
    normal = glm::dvec3(0, 1, 0);
  }
  fc = v->pos + glm::dvec3(0, -halfSize, 0);
  dist = glm::distance(fc, intersectionPoint);
  if (dist < minDist) {
    minDist = dist;
    normal = glm::dvec3(0, -1, 0);
  }
  fc = v->pos + glm::dvec3(0, 0, halfSize);
  dist = glm::distance(fc, intersectionPoint);
  if (dist < minDist) {
    minDist = dist;
    normal = glm::dvec3(0, 0, 1);
  }
  fc = v->pos + glm::dvec3(0, 0, -halfSize);
  dist = glm::distance(fc, intersectionPoint);
  if (dist < minDist) {
    minDist = dist;
    normal = glm::dvec3(0, 0, -1);
  }

  double const angle = glm::angle(normal, rayDir);
  return (angle > PI_HALF) ? M_PI - angle : angle;
}

void
VoxelView::updateAabbCache() const
{
  if (owner == nullptr || index >= owner->voxels.aabb_min.size())
    return;
  aabb_cache =
    AABB(owner->voxels.aabb_min[index], owner->voxels.aabb_max[index]);
}

// *** DetailedVoxelView *** //
Primitive*
DetailedVoxelView::clone()
{
  if (owner == nullptr || index >= owner->voxels.centers.size())
    return nullptr;

  std::vector<int> int_values;
  std::vector<double> double_values;
  if (index < owner->detailed_voxels.int_values.size())
    int_values = owner->detailed_voxels.int_values[index];
  if (index < owner->detailed_voxels.double_values.size())
    double_values = owner->detailed_voxels.double_values[index];

  DetailedVoxel* dv = new DetailedVoxel(owner->voxels.centers[index].pos.x,
                                        owner->voxels.centers[index].pos.y,
                                        owner->voxels.centers[index].pos.z,
                                        owner->voxels.half_size[index],
                                        std::move(int_values),
                                        std::move(double_values));
  if (index < owner->detailed_voxels.max_pad.size())
    dv->setMaxPad(owner->detailed_voxels.max_pad[index]);
  if (index < owner->voxels.materials.size())
    dv->material = owner->voxels.materials[index];
  dv->update();
  return dv;
}

IntersectionHandlingResult
DetailedVoxelView::onRayIntersection(NoiseSource<double>& uniformNoiseSource,
                                     glm::dvec3& rayDirection,
                                     glm::dvec3 const& insideIntersectionPoint,
                                     glm::dvec3 const& outsideIntersectionPoint,
                                     double rayIntensity)
{
  std::string mode = part->onRayIntersectionMode;
  if (mode == "FIXED") {
    return onRayIntersectionFixed(uniformNoiseSource,
                                  rayDirection,
                                  insideIntersectionPoint,
                                  outsideIntersectionPoint,
                                  rayIntensity,
                                  part->onRayIntersectionArgument);
  } else if (mode == "SCALED") {
    return onRayIntersectionScaled(uniformNoiseSource,
                                   rayDirection,
                                   insideIntersectionPoint,
                                   outsideIntersectionPoint,
                                   rayIntensity,
                                   part->onRayIntersectionArgument);
  }

  return onRayIntersectionTransmittive(uniformNoiseSource,
                                       rayDirection,
                                       insideIntersectionPoint,
                                       outsideIntersectionPoint,
                                       rayIntensity);
}

void
DetailedVoxelView::onFinishLoading(NoiseSource<double>& uniformNoiseSource)
{
  if (owner == nullptr || part == nullptr)
    return;
  if (part->onRayIntersectionMode != "SCALED")
    return;

  double pad = getDoubleValueByIndex(0);
  double maxPad = 0.0;
  if (index < owner->detailed_voxels.max_pad.size())
    maxPad = owner->detailed_voxels.max_pad[index];
  if (maxPad <= 0.0)
    return;

  double const halfSize = owner->voxels.half_size[index];
  double newHalfSize =
    halfSize * std::pow(pad / maxPad, part->onRayIntersectionArgument);

  if (part->randomShift) {
    double maxShift = halfSize - newHalfSize;
    if (std::isnan(maxShift))
      maxShift = 0.0;
    glm::dvec3 shift(uniformNoiseSource.next() * maxShift,
                     uniformNoiseSource.next() * maxShift,
                     uniformNoiseSource.next() * maxShift);
    owner->voxels.centers[index].pos += shift;
  }

  owner->voxels.half_size[index] = newHalfSize;
  update();
}

bool
DetailedVoxelView::canComputeSigmaWithLadLut()
{
  return part != nullptr && part->ladlut != nullptr;
}

double
DetailedVoxelView::computeSigmaWithLadLut(glm::dvec3 const& direction)
{
  if (part == nullptr || part->ladlut == nullptr)
    return std::numeric_limits<double>::quiet_NaN();
  double pad = getDoubleValueByIndex(0);
  return part->ladlut->computeSigma(pad, direction.x, direction.y, direction.z);
}

IntersectionHandlingResult
DetailedVoxelView::onRayIntersectionTransmittive(
  NoiseSource<double>& uniformNoiseSource,
  glm::dvec3& rayDirection,
  glm::dvec3 const& insideIntersectionPoint,
  glm::dvec3 const& outsideIntersectionPoint,
  double rayIntensity)
{
  double intersectionLength =
    glm::distance(insideIntersectionPoint, outsideIntersectionPoint);
  double sigma = getDoubleValueByIndex(0);

  if (part != nullptr && part->ladlut != nullptr) {
    sigma = part->ladlut->computeSigma(
      sigma, rayDirection.x, rayDirection.y, rayDirection.z);
  }

  if (std::isnan(sigma) || sigma == 0.0) {
    return IntersectionHandlingResult(insideIntersectionPoint, true);
  }

  double const rndProb = uniformNoiseSource.next();
  double const s = -std::log(rndProb) / sigma;
  if (s > intersectionLength) {
    return IntersectionHandlingResult(insideIntersectionPoint, true);
  } else {
    glm::dvec3 nip = insideIntersectionPoint + rayDirection * s;
    return IntersectionHandlingResult(nip, false);
  }
}

IntersectionHandlingResult
DetailedVoxelView::onRayIntersectionScaled(
  NoiseSource<double>& uniformNoiseSource,
  glm::dvec3& rayDirection,
  glm::dvec3 const& insideIntersectionPoint,
  glm::dvec3 const& outsideIntersectionPoint,
  double rayIntensity,
  double scaleFactor)
{
  double transmittance = getDoubleValueByIndex(8);
  return IntersectionHandlingResult(
    insideIntersectionPoint, transmittance == 1.0 || std::isnan(transmittance));
}

IntersectionHandlingResult
DetailedVoxelView::onRayIntersectionFixed(
  NoiseSource<double>& uniformNoiseSource,
  glm::dvec3& rayDirection,
  glm::dvec3 const& insideIntersectionPoint,
  glm::dvec3 const& outsideIntersectionPoint,
  double rayIntensity,
  double fixedSize)
{
  double transmittance = getDoubleValueByIndex(8);
  return IntersectionHandlingResult(
    insideIntersectionPoint, transmittance == 1.0 || std::isnan(transmittance));
}

double
DetailedVoxelView::getDoubleValueByIndex(size_t idx) const
{
  if (owner == nullptr)
    return std::numeric_limits<double>::quiet_NaN();
  if (index >= owner->detailed_voxels.double_values.size())
    return std::numeric_limits<double>::quiet_NaN();
  std::vector<double> const& vals = owner->detailed_voxels.double_values[index];
  if (idx >= vals.size())
    return std::numeric_limits<double>::quiet_NaN();
  return vals[idx];
}
