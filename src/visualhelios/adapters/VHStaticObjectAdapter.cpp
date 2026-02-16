#ifdef PCL_BINDING

#include <AABB.h>
#include <VHStaticObjectAdapter.h>
#include <util/HeliosException.h>

#include <functional>
#include <vector>

using visualhelios::VHStaticObjectAdapter;

// ***  BUILDING  *** //
// ****************** //
void
VHStaticObjectAdapter::buildPolymesh()
{
  // Select adequate function to add geometry to polymesh.
  std::function<void(std::size_t, int&)> addGeometryToPolymesh;
  if (staticObj.getGeometryType() == ScenePart::GeometryType::TRIANGLE) {
    addGeometryToPolymesh = [&](std::size_t geometryIndex,
                                int& offset) -> void {
      addTriangleToPolymesh(geometryIndex, offset);
    };
  } else if (staticObj.getGeometryType() == ScenePart::GeometryType::VOXEL ||
             staticObj.getGeometryType() ==
               ScenePart::GeometryType::DETAILED_VOXEL) {
    addGeometryToPolymesh = [&](std::size_t geometryIndex,
                                int& offset) -> void {
      addVoxelToPolymesh(geometryIndex, offset);
    };
  } else {
    throw HeliosException("VHStaticObjectAdapter::buildPolymesh failed.\n"
                          "Geometry type cannot be recognized");
  }

  // Instantiate a new polymesh replacing the old one, if any
  constructPolymesh();
  vertices.clear();

  // Add each geometry element to the polymesh.
  int offset = 0; // To handle vertex indices
  std::size_t const count = staticObj.geometryCount();
  for (std::size_t i = 0; i < count; ++i) {
    addGeometryToPolymesh(i, offset);
  }
}

// ***  UTILS  *** //
// *************** //
void
VHStaticObjectAdapter::addTriangleToPolymesh(std::size_t geometryIndex,
                                             int& offset)
{
  pcl::Vertices verts;
  std::size_t const n = staticObj.geometryDynamicVertexCount(geometryIndex);
  if (n < 3) {
    return;
  }
  for (int i = 0; i < 3; ++i) {
    Vertex vertex;
    vertex.pos = staticObj.geometryDynamicVertexPosition(geometryIndex, i);
    vertex.normal = staticObj.geometryDynamicVertexNormal(geometryIndex, i);
    vertexToMesh(vertex);
    verts.vertices.push_back(offset + i);
  }
  vertices.push_back(verts);
  offset += 3;
}
void
VHStaticObjectAdapter::addVoxelToPolymesh(std::size_t geometryIndex,
                                          int& offset)
{
  std::shared_ptr<AABB> box = staticObj.geometryAABB(geometryIndex);
  if (box == nullptr) {
    return;
  }
  glm::dvec3 const mn = box->getMin();
  glm::dvec3 const mx = box->getMax();
  Vertex A;
  A.pos = glm::dvec3(mn.x, mn.y, mn.z);
  Vertex B;
  B.pos = glm::dvec3(mx.x, mn.y, mn.z);
  Vertex C;
  C.pos = glm::dvec3(mn.x, mn.y, mx.z);
  Vertex D;
  D.pos = glm::dvec3(mx.x, mn.y, mx.z);
  Vertex E;
  E.pos = glm::dvec3(mn.x, mx.y, mn.z);
  Vertex F;
  F.pos = glm::dvec3(mx.x, mx.y, mn.z);
  Vertex G;
  G.pos = glm::dvec3(mn.x, mx.y, mx.z);
  Vertex H;
  H.pos = glm::dvec3(mx.x, mx.y, mx.z);

  // Put all vertices into the mesh
  int const base = offset;
  vertexToMesh(A);
  vertexToMesh(B);
  vertexToMesh(C);
  vertexToMesh(D);
  vertexToMesh(E);
  vertexToMesh(F);
  vertexToMesh(G);
  vertexToMesh(H);
  offset += 8;

  auto addFace = [&](std::initializer_list<int> idxs) {
    pcl::Vertices face;
    face.vertices.insert(face.vertices.end(), idxs.begin(), idxs.end());
    vertices.push_back(face);
  };
  addFace({ base + 0, base + 4, base + 5, base + 1, base + 0 }); // Lower
  addFace({ base + 0, base + 1, base + 3, base + 2, base + 0 }); // Front
  addFace({ base + 0, base + 4, base + 6, base + 2, base + 0 }); // Left
  addFace({ base + 4, base + 6, base + 7, base + 5, base + 4 }); // Back
  addFace({ base + 1, base + 3, base + 7, base + 5, base + 1 }); // Right
  addFace({ base + 2, base + 3, base + 7, base + 6, base + 2 }); // Upper
}

#endif
