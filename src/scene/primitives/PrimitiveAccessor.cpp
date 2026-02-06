#include <scene/primitives/PrimitiveAccessor.h>

#include <MathConstants.h>
#include <Triangle.h>
#include <Voxel.h>

Vertex*
PrimitiveAccessor::getVertices(PrimitiveRef const& ref)
{
  if (ref.part == nullptr)
    return nullptr;
  if (ref.type == ScenePart::PrimitiveType::TRIANGLE) {
    std::size_t const base = 3 * ref.index;
    if (base + 2 >= ref.part->triangles.vertices.size())
      return nullptr;
    return &ref.part->triangles.vertices[base];
  }
  if (ref.type == ScenePart::PrimitiveType::VOXEL) {
    if (ref.index >= ref.part->voxels.centers.size())
      return nullptr;
    return &ref.part->voxels.centers[ref.index];
  }
  return nullptr;
}

glm::dvec3
PrimitiveAccessor::getCentroid(PrimitiveRef const& ref)
{
  if (ref.type == ScenePart::PrimitiveType::TRIANGLE) {
    Vertex* verts = getVertices(ref);
    if (verts == nullptr)
      return glm::dvec3(0.0, 0.0, 0.0);
    return (verts[0].pos + verts[1].pos + verts[2].pos) / 3.0;
  }
  if (ref.type == ScenePart::PrimitiveType::VOXEL) {
    Vertex* v = getVertices(ref);
    if (v == nullptr)
      return glm::dvec3(0.0, 0.0, 0.0);
    return v->pos;
  }
  return glm::dvec3(0.0, 0.0, 0.0);
}

AABB
PrimitiveAccessor::getAABB(PrimitiveRef const& ref)
{
  if (ref.part == nullptr)
    return AABB();

  if (ref.type == ScenePart::PrimitiveType::TRIANGLE) {
    if (ref.index < ref.part->triangles.aabb_min.size() &&
        ref.index < ref.part->triangles.aabb_max.size()) {
      return AABB(ref.part->triangles.aabb_min[ref.index],
                  ref.part->triangles.aabb_max[ref.index]);
    }
    Vertex* verts = getVertices(ref);
    if (verts == nullptr)
      return AABB();
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
    return AABB(glm::dvec3(minX, minY, minZ), glm::dvec3(maxX, maxY, maxZ));
  }

  if (ref.type == ScenePart::PrimitiveType::VOXEL) {
    if (ref.index < ref.part->voxels.aabb_min.size() &&
        ref.index < ref.part->voxels.aabb_max.size()) {
      return AABB(ref.part->voxels.aabb_min[ref.index],
                  ref.part->voxels.aabb_max[ref.index]);
    }
    if (ref.index >= ref.part->voxels.centers.size() ||
        ref.index >= ref.part->voxels.half_size.size()) {
      return AABB();
    }
    glm::dvec3 hs(ref.part->voxels.half_size[ref.index],
                  ref.part->voxels.half_size[ref.index],
                  ref.part->voxels.half_size[ref.index]);
    glm::dvec3 min = ref.part->voxels.centers[ref.index].pos - hs;
    glm::dvec3 max = ref.part->voxels.centers[ref.index].pos + hs;
    return AABB(min, max);
  }

  return AABB();
}

double
PrimitiveAccessor::getRayIntersectionDistance(PrimitiveRef const& ref,
                                              glm::dvec3 const& rayOrigin,
                                              glm::dvec3 const& rayDir)
{
  if (ref.part == nullptr)
    return -1.0;

  if (ref.type == ScenePart::PrimitiveType::TRIANGLE) {
    glm::dvec3 e1, e2, v0;
    double eps = 0.0000001;
    if (ref.index < ref.part->triangles.e1.size())
      e1 = ref.part->triangles.e1[ref.index];
    if (ref.index < ref.part->triangles.e2.size())
      e2 = ref.part->triangles.e2[ref.index];
    if (ref.index < ref.part->triangles.v0.size())
      v0 = ref.part->triangles.v0[ref.index];
    if (ref.index < ref.part->triangles.eps.size())
      eps = ref.part->triangles.eps[ref.index];
    if (ref.part->triangles.e1.size() <= ref.index ||
        ref.part->triangles.e2.size() <= ref.index ||
        ref.part->triangles.v0.size() <= ref.index) {
      Vertex* verts = getVertices(ref);
      if (verts == nullptr)
        return -1.0;
      v0 = verts[0].pos;
      e1 = verts[1].pos - v0;
      e2 = verts[2].pos - v0;
    }

    glm::dvec3 const h = glm::cross(rayDir, e2);
    double const a = e1.x * h.x + e1.y * h.y + e1.z * h.z;

    if (a > -eps && a < eps)
      return -1.0;

    double const f = 1.0 / a;
    glm::dvec3 const s = rayOrigin - v0;
    double const u = f * (s.x * h.x + s.y * h.y + s.z * h.z);

    if (u < 0.0 || u > 1.0)
      return -1.0;

    glm::dvec3 const q = glm::cross(s, e1);
    double const v = f * (rayDir.x * q.x + rayDir.y * q.y + rayDir.z * q.z);

    if (v < 0.0 || u + v > 1.0)
      return -1.0;

    double const t = f * (e2.x * q.x + e2.y * q.y + e2.z * q.z);

    if (t > eps)
      return t;
    return -1.0;
  }

  if (ref.type == ScenePart::PrimitiveType::VOXEL) {
    AABB box = getAABB(ref);
    return box.getRayIntersectionDistance(rayOrigin, rayDir);
  }

  return -1.0;
}

std::vector<double>
PrimitiveAccessor::getRayIntersection(PrimitiveRef const& ref,
                                      glm::dvec3 const& rayOrigin,
                                      glm::dvec3 const& rayDir)
{
  if (ref.part == nullptr)
    return std::vector<double>{ -1 };

  if (ref.type == ScenePart::PrimitiveType::TRIANGLE) {
    glm::dvec3 e1, e2, v0;
    double eps = 0.0000001;
    if (ref.index < ref.part->triangles.e1.size())
      e1 = ref.part->triangles.e1[ref.index];
    if (ref.index < ref.part->triangles.e2.size())
      e2 = ref.part->triangles.e2[ref.index];
    if (ref.index < ref.part->triangles.v0.size())
      v0 = ref.part->triangles.v0[ref.index];
    if (ref.index < ref.part->triangles.eps.size())
      eps = ref.part->triangles.eps[ref.index];
    if (ref.part->triangles.e1.size() <= ref.index ||
        ref.part->triangles.e2.size() <= ref.index ||
        ref.part->triangles.v0.size() <= ref.index) {
      Vertex* verts = getVertices(ref);
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

  if (ref.type == ScenePart::PrimitiveType::VOXEL) {
    AABB box = getAABB(ref);
    return box.getRayIntersection(rayOrigin, rayDir);
  }

  return std::vector<double>{ -1 };
}

double
PrimitiveAccessor::getGroundZOffset(PrimitiveRef const& ref)
{
  if (ref.part == nullptr)
    return 0.0;
  if (ref.type == ScenePart::PrimitiveType::VOXEL) {
    if (ref.index >= ref.part->voxels.half_size.size())
      return 0.0;
    return ref.part->voxels.half_size[ref.index] * 2.0;
  }
  return 0.0;
}

std::shared_ptr<Material>
PrimitiveAccessor::getMaterial(PrimitiveRef const& ref)
{
  if (ref.part == nullptr)
    return nullptr;
  if (ref.type == ScenePart::PrimitiveType::TRIANGLE) {
    if (ref.index < ref.part->triangles.materials.size())
      return ref.part->triangles.materials[ref.index];
    return nullptr;
  }
  if (ref.type == ScenePart::PrimitiveType::VOXEL) {
    if (ref.index < ref.part->voxels.materials.size())
      return ref.part->voxels.materials[ref.index];
    return nullptr;
  }
  return nullptr;
}

bool
PrimitiveAccessor::isGround(PrimitiveRef const& ref)
{
  std::shared_ptr<Material> mat = getMaterial(ref);
  return mat != nullptr && mat->isGround;
}

Primitive*
PrimitiveAccessor::getPrimitiveView(PrimitiveRef const& ref)
{
  if (ref.part == nullptr)
    return nullptr;
  if (ref.index >= ref.part->mPrimitives.size())
    return nullptr;
  return ref.part->mPrimitives[ref.index];
}
