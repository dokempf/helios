#ifdef PCL_BINDING

#include <VHStaticObjectAdapter.h>

#include <functional>

using visualhelios::VHStaticObjectAdapter;

// ***  BUILDING  *** //
// ****************** //
void
VHStaticObjectAdapter::buildPolymesh()
{
  // Instantiate a new polymesh replacing the old one, if any
  constructPolymesh();
  vertices.clear();

  // Add each primitive to the polymesh
  int offset = 0; // To handle vertex indices
  if (staticObj.getPrimitiveType() == ScenePart::PrimitiveType::TRIANGLE) {
    size_t const n = staticObj.triangles.size();
    for (size_t i = 0; i < n; ++i) {
      size_t const base = 3 * i;
      if (base + 2 >= staticObj.triangles.vertices.size())
        break;
      addTriangleToPolymesh(&staticObj.triangles.vertices[base], offset);
    }
  } else if (staticObj.getPrimitiveType() == ScenePart::PrimitiveType::VOXEL) {
    size_t const n = staticObj.voxels.size();
    for (size_t i = 0; i < n; ++i) {
      if (i >= staticObj.voxels.centers.size() ||
          i >= staticObj.voxels.half_size.size())
        break;
      addVoxelToPolymesh(
        staticObj.voxels.centers[i], staticObj.voxels.half_size[i], offset);
    }
  } else {
    throw HeliosException("VHStaticObjectAdapter::buildPolymesh failed.\n"
                          "Type of primitive cannot be recognized");
  }
}

// ***  UTILS  *** //
// *************** //
void
VHStaticObjectAdapter::addTriangleToPolymesh(Vertex const* vertices,
                                             int& offset)
{
  pcl::Vertices verts; // Vertex connection order for the primitive
  for (int i = 0; i < 3; ++i) {
    vertexToMesh(vertices[i]);
    verts.vertices.push_back(offset + i); // Register vertex order
  }
  vertices.push_back(verts); // Register primitive order of vertices
  offset += 3;               // Update vertex start index for next primitive
}
void
VHStaticObjectAdapter::addVoxelToPolymesh(Vertex const& center,
                                          double halfSize,
                                          int& offset)
{
  // Get all vertices from voxel
  double const fullSize = 2.0 * halfSize;
  glm::dvec3 halfVec(halfSize, halfSize, halfSize);
  Vertex A;
  A.pos = center.pos - halfVec;
  Vertex B;
  B.pos = A.pos + glm::dvec3(fullSize, 0, 0);
  Vertex C;
  C.pos = A.pos + glm::dvec3(0, 0, fullSize);
  Vertex D;
  D.pos = B.pos + glm::dvec3(0, 0, fullSize);
  Vertex E;
  E.pos = A.pos + glm::dvec3(0, fullSize, 0);
  Vertex F;
  F.pos = E.pos + glm::dvec3(fullSize, 0, 0);
  Vertex G;
  G.pos = E.pos + glm::dvec3(0, 0, fullSize);
  Vertex H;
  H.pos = center.pos + halfVec;

  // Put all vertices into the mesh
  vertexToMesh(A);
  vertexToMesh(B);
  vertexToMesh(C);
  vertexToMesh(D);
  vertexToMesh(E);
  vertexToMesh(F);
  vertexToMesh(G);
  vertexToMesh(H);
  offset += 8; // Update vertex start index for next primitive

  // Register vertex order
  // Lower face
  pcl::Vertices lower; // Vertex connection order for the lower face
  lower.vertices.push_back(offset);
  lower.vertices.push_back(offset + 4);
  lower.vertices.push_back(offset + 5);
  lower.vertices.push_back(offset + 1);
  lower.vertices.push_back(offset);
  vertices.push_back(lower); // Register lower face
  // Front face
  pcl::Vertices front; // Vertex connection order for the front face
  front.vertices.push_back(offset);
  front.vertices.push_back(offset + 1);
  front.vertices.push_back(offset + 3);
  front.vertices.push_back(offset + 2);
  front.vertices.push_back(offset);
  vertices.push_back(front); // Register front face
  // Left face
  pcl::Vertices left; // Vertex connection order for the left face
  left.vertices.push_back(offset);
  left.vertices.push_back(offset + 4);
  left.vertices.push_back(offset + 6);
  left.vertices.push_back(offset + 2);
  left.vertices.push_back(offset);
  vertices.push_back(left); // Register left face
  // Back face
  pcl::Vertices back; // Vertex connection order for the back face
  back.vertices.push_back(offset + 4);
  back.vertices.push_back(offset + 6);
  back.vertices.push_back(offset + 7);
  back.vertices.push_back(offset + 5);
  back.vertices.push_back(offset + 4);
  vertices.push_back(back); // Register back face
  // Right face
  pcl::Vertices right; // Vertex connection order for the right face
  right.vertices.push_back(offset + 1);
  right.vertices.push_back(offset + 3);
  right.vertices.push_back(offset + 7);
  right.vertices.push_back(offset + 5);
  right.vertices.push_back(offset + 1);
  vertices.push_back(right); // Register right face
  // Upper face
  pcl::Vertices upper; // Vertex connection order for the upper face
  upper.vertices.push_back(offset + 2);
  upper.vertices.push_back(offset + 3);
  upper.vertices.push_back(offset + 7);
  upper.vertices.push_back(offset + 6);
  upper.vertices.push_back(offset + 2);
  vertices.push_back(upper); // Register upper face
}

#endif
