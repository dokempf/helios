#ifdef PCL_BINDING

#include <Primitive.h>
#include <visualhelios/VHSimpleCanvas.h>

#include <sstream>

using visualhelios::VHSimpleCanvas;

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
VHSimpleCanvas::VHSimpleCanvas(string const title)
  : VHNormalsCanvas(title, true, true, false, 2.0f)
{
}

// ***  CANVAS METHODS  *** //
// ************************ //
void
VHSimpleCanvas::configure()
{
  // Configure base canvas
  VHNormalsCanvas::configure();

  // Configure camera
  viewer->setCameraPosition(0, -75.0, 35.0, 0.0, 1.0, 0.0);
}

void
VHSimpleCanvas::start()
{
  // Start base canvas
  VHNormalsCanvas::start();

  // Build polygon meshes and add them to viewer
  for (shared_ptr<VHDynObjectXYZRGBAdapter>& dynObj : dynObjs) {
    dynObj->buildPolymesh();
    viewer->addPolygonMesh<pcl::PointXYZRGB>(
      dynObj->getPolymesh(), dynObj->getVertices(), dynObj->getId());
  }

  // Apply initial transformations
  for (shared_ptr<VHDynObjectXYZRGBAdapter>& dynObj : dynObjs) {
    dynObj->doStep();
    viewer->updatePolygonMesh<pcl::PointXYZRGB>(
      dynObj->getPolymesh(), dynObj->getVertices(), dynObj->getId());
    // Render initial normals if requested
    if (isRenderingNormals())
      renderNormals(*dynObj);
  }
}

void
VHSimpleCanvas::update()
{
  // Base canvas update
  VHNormalsCanvas::update();

  // Apply dynamic update function to primitives
  if (dynamicUpdateFunction)
    dynamicUpdateFunction(dynObjs);

  // Update polygon meshes after applying dynamic function if necessary
  for (shared_ptr<VHDynObjectXYZRGBAdapter>& dynObj : dynObjs) {
    // Continue to next iteration if no updates are needed for this
    if (!dynObj->doStep(true, false) && !isNeedingUpdate())
      continue;
    // Update the polygon mesh itself
    viewer->updatePolygonMesh<pcl::PointXYZRGB>(
      dynObj->getPolymesh(), dynObj->getVertices(), dynObj->getId());
    // Update normals if requested
    if (isRenderingNormals())
      renderNormals(*dynObj);
  }
}

// ***  NORMALS RENDERING METHODS  ***  //
// ************************************ //
void
VHSimpleCanvas::renderNormals(VHStaticObjectAdapter& staticObj)
{
  if (!staticObj.isRenderingNormals())
    return;
  ScenePart& sp = staticObj.getStaticObj();
  size_t numPrims = 0;
  if (sp.getPrimitiveType() == ScenePart::PrimitiveType::TRIANGLE)
    numPrims = sp.triangles.size();
  else if (sp.getPrimitiveType() == ScenePart::PrimitiveType::VOXEL)
    numPrims = sp.voxels.size();
  for (size_t i = 0; i < numPrims; ++i) {
    float nx = 0;
    float ny = 0;
    float nz = 0;
    pcl::PointXYZ p, q;
    size_t numVertices = 0;
    if (sp.getPrimitiveType() == ScenePart::PrimitiveType::TRIANGLE) {
      size_t const base = 3 * i;
      if (base + 2 >= sp.triangles.vertices.size())
        continue;
      for (size_t j = 0; j < 3; ++j) {
        Vertex& vertex = sp.triangles.vertices[base + j];
        nx += vertex.normal.x;
        ny += vertex.normal.y;
        nz += vertex.normal.z;
        p.x += vertex.pos.x;
        p.y += vertex.pos.y;
        p.z += vertex.pos.z;
      }
      numVertices = 3;
    } else if (sp.getPrimitiveType() == ScenePart::PrimitiveType::VOXEL) {
      if (i >= sp.voxels.centers.size())
        continue;
      Vertex& vertex = sp.voxels.centers[i];
      nx += vertex.normal.x;
      ny += vertex.normal.y;
      nz += vertex.normal.z;
      p.x += vertex.pos.x;
      p.y += vertex.pos.y;
      p.z += vertex.pos.z;
      numVertices = 1;
    }
    if (numVertices == 0)
      continue;
    float const numVerticesf = (float)numVertices;
    nx /= numVerticesf;
    ny /= numVerticesf;
    nz /= numVerticesf;
    p.x /= numVerticesf;
    p.y /= numVerticesf;
    p.z /= numVerticesf;
    q.x = p.x + getNormalMagnitude() * nx;
    q.y = p.y + getNormalMagnitude() * ny;
    q.z = p.z + getNormalMagnitude() * nz;
    std::stringstream ss;
    ss << staticObj.getId() << "_normal" << i;
    viewer->removeShape(ss.str());
    viewer->addLine(
      p, q, normalDefColor[0], normalDefColor[1], normalDefColor[2], ss.str());
  }
}

void
VHSimpleCanvas::unrenderAllNormals()
{
  for (shared_ptr<VHDynObjectXYZRGBAdapter> const& dynObj : dynObjs) {
    if (!dynObj->isRenderingNormals())
      continue; // Skip, nothing to remove
    ScenePart& sp = dynObj->getDynObj();
    size_t numPrims = 0;
    if (sp.getPrimitiveType() == ScenePart::PrimitiveType::TRIANGLE)
      numPrims = sp.triangles.size();
    else if (sp.getPrimitiveType() == ScenePart::PrimitiveType::VOXEL)
      numPrims = sp.voxels.size();
    for (size_t i = 0; i < numPrims; ++i) {
      std::stringstream ss;
      ss << dynObj->getId() << "_normal" << i;
      viewer->removeShape(ss.str());
    }
  }
}

#endif
