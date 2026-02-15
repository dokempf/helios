#include "GeoTiffFileLoader.h"
#include <Vertex.h>
#include <boost/variant/get.hpp>
#include <fstream>
#include <glm/geometric.hpp>
#include <logging.hpp>
#include <ogrsf_frmts.h>
#include <sstream>

// ***  R U N  *** //
// *************** //
ScenePart*
GeoTiffFileLoader::run()
{
  std::string const& filePathString =
    boost::get<std::string const&>(params["filepath"]);
  std::stringstream ss;
  ss << "Reading 3D model from GeoTiff file '" << filePathString << "'...";
  logging::INFO(ss.str());
  ss.str("");

  std::fstream file;
  GDALDataset* tiff;

  try {
    // Extract data from tiff file
    file.open(filePathString, std::fstream::in);
    tiff = (GDALDataset*)GDALOpen(filePathString.data(), GA_ReadOnly);

    // Obtain coordinate reference system, layer, raster and envelope
    obtainCRS(tiff);
    obtainLayer(tiff);
    obtainRaster(tiff);
    obtainEnvelope(tiff);

    // Fill vertices and build triangles
    fillVertices();
    buildTriangles();
    loadMaterial();

    // Finish primsOut
    primsOut->mEnv = env;

    // Release vertices
    releaseVertices();
  } catch (std::exception& e) {
    ss << "File (" << filePathString << ") could not be successfully read."
       << "\n\tException: " << e.what() << "\n"
       << "Aborting load attempt.";
    logging::ERR(ss.str());
    exit(1);
  }

  // Close tiff
  GDALClose(tiff);

  // Return
  return primsOut;
}

// ***  METHODS  *** //
// ***************** //
void
GeoTiffFileLoader::obtainCRS(GDALDataset* tiff)
{
  const OGRSpatialReference* crs = tiff->GetSpatialRef();
  std::shared_ptr<OGRSpatialReference> crsDefault = nullptr;
  if (crs != nullptr) {
    sourceCRS = std::shared_ptr<OGRSpatialReference>(crs->Clone());
  } else {
    crsDefault =
      std::shared_ptr<OGRSpatialReference>(new OGRSpatialReference());
    sourceCRS = std::shared_ptr<OGRSpatialReference>(crsDefault);
    std::stringstream ss;
    ss << "WARNING (GeoTiffFileLoader.run):\n\t"
       << "No coordinate reference system was available.\n\t"
          "Using default one. ";
    logging::WARN(ss.str());
  }
}

void
GeoTiffFileLoader::obtainLayer(GDALDataset* tiff)
{
  layer = tiff->GetLayer(0); // Start at 0
}

void
GeoTiffFileLoader::obtainRaster(GDALDataset* tiff)
{
  raster = tiff->GetRasterBand(1);
  rasterWidth = raster->GetXSize();
  rasterHeight = raster->GetYSize();
}

void
GeoTiffFileLoader::obtainEnvelope(GDALDataset* tiff)
{
  // Get geometry if available
  OGRGeometry* geom = nullptr;
  if (layer != nullptr) {
    geom = layer->GetSpatialFilter();
  }

  // Build initial envelope
  env = new OGREnvelope();

  if (geom != nullptr) { // Envelope from geometry
    geom->getEnvelope(env);
  } else if (layer != nullptr) { //
    std::stringstream ss;
    if (layer->GetExtent(env, true) != OGRERR_NONE) {
      ss << "ERROR at GeoTiffFileLoader::run when "
         << "retrieving envelope from layer.";
      logging::ERR(ss.str());
      exit(1);
    }
    ss << "WARNING! Unexpected secenario at GeoTiffFileLoader when "
       << "obtaining envelope";
    logging::WARN(ss.str());
  } else { // Envelope from GeoTransform
    double transform[6];
    tiff->GetGeoTransform(transform);
    if (transform[1] > 0.0) {
      env->MinX = transform[0];
      env->MaxX = env->MinX + transform[1] * rasterWidth;
    } else {
      env->MaxX = transform[0];
      env->MinX = env->MaxX + transform[2] * rasterWidth;
    }
    if (transform[4] > 0.0) {
      env->MinY = transform[3];
      env->MaxY = env->MinY + transform[4] * rasterHeight;
    } else {
      env->MaxY = transform[3];
      env->MinY = env->MaxY + transform[5] * rasterHeight;
    }

    std::stringstream ss;
    ss << "GeoTiffFileLoader is using envelope from GeoTransform "
       << "because a layer-based one was not found.\n"
       << "\t(x,y) boundaries go from (" << env->MinX << "," << env->MinY
       << ") to (" << env->MaxX << "," << env->MaxY << ")";
    logging::INFO(ss.str());
    ss.str("");
  }
  minx = env->MinX;
  miny = env->MinY;
  width = env->MaxX - env->MinX;
  height = env->MaxY - env->MinY;
  pixelWidth = width / rasterWidth;
  pixelHeight = height / rasterHeight;

  // Envelope logging
  std::stringstream ss;
  ss << "GeoTiffLoader envelope description:\n"
     << "\tenvelope dimensions " << width << " x " << height << "\n"
     << "\traster dimensions " << rasterWidth << " x " << rasterHeight << "\n"
     << "\tpixel dimension " << pixelWidth << " x " << pixelHeight;
  logging::INFO(ss.str());
}

void
GeoTiffFileLoader::fillVertices()
{
  float z;
  float nodata = raster->GetNoDataValue();
  Vertex* v;
  vertices = new Vertex**[rasterWidth];
  double halfPixelWidth = pixelWidth / 2.0;
  double halfPixelHeight = pixelHeight / 2.0;
  for (int x = 0; x < rasterWidth; x++) {
    vertices[x] = new Vertex*[rasterHeight];
    for (int y = 0; y < rasterHeight; y++) {
      if (raster->RasterIO(GF_Read,
                           x, // Pixel x-offset, starting at top left corner
                           y, // Pixel y-offset, starting at top left corner
                           1,
                           1,
                           &z,
                           1, // Buffer image width
                           1, // Buffer image height
                           GDT_Float32,
                           0,      // Pixel offset for &z
                           0,      // Scanline offset for &z,
                           nullptr // Extra args
                           ) == CE_Failure) {
        std::stringstream ss;
        ss << "Raster failure at GeoTiffFileLoader::filVertices\n\t"
           << "(" << x << ", " << y << ")" << std::endl;
        logging::WARN(ss.str());
      }
      if (std::abs(z - nodata) < eps) {
        v = nullptr;
      } else {
        v = new Vertex();
        v->pos = glm::dvec3(minx + x * pixelWidth + halfPixelWidth,
                            miny - y * pixelHeight - halfPixelHeight + height,
                            z);
        v->texcoords =
          glm::dvec2(((double)x) / ((double)rasterWidth),
                     ((double)(rasterHeight - y)) / ((double)rasterHeight));
      }
      vertices[x][y] = v;
    }
  }
}

void
GeoTiffFileLoader::loadMaterial()
{
  // Parse material
  std::vector<std::shared_ptr<Material>> matvec = parseMaterials();
  std::shared_ptr<Material> mat;
  if (matvec.empty())
    mat = getMaterial("default");
  else
    mat = matvec[0];

  // Assign material
  for (std::size_t i = 0; i < primsOut->geometryCount(); ++i) {
    primsOut->setGeometryMaterial(i, mat);
  }
}

void
GeoTiffFileLoader::releaseVertices()
{
  for (int x = 0; x < rasterWidth; x++) {
    for (int y = 0; y < rasterHeight; y++) {
      if (vertices[x][y] != nullptr) {
        delete vertices[x][y];
      }
    }
  }
}

void
GeoTiffFileLoader::buildTriangles()
{
  TriangleScenePart* const trianglePart =
    dynamic_cast<TriangleScenePart*>(primsOut);
  if (trianglePart == nullptr) {
    return;
  }

  auto hasTriangle = [](Vertex* a, Vertex* b, Vertex* c) {
    return a != nullptr && b != nullptr && c != nullptr;
  };

  std::size_t triangleCount = 0;
  for (int x = 0; x < rasterWidth - 1; ++x) {
    for (int y = 0; y < rasterHeight - 1; ++y) {
      Vertex* const vert0 = vertices[x][y];
      Vertex* const vert1 = vertices[x][y + 1];
      Vertex* const vert2 = vertices[x + 1][y + 1];
      Vertex* const vert3 = vertices[x + 1][y];
      triangleCount += hasTriangle(vert0, vert1, vert3) ? 1 : 0;
      triangleCount += hasTriangle(vert1, vert2, vert3) ? 1 : 0;
    }
  }

  trianglePart->vertices.set_size(triangleCount, 9);
  trianglePart->normals.zeros(triangleCount, 9);
  trianglePart->materialIndex.zeros(triangleCount);
  trianglePart->materialTable.clear();

  auto writeTriangle =
    [&](std::size_t row, Vertex const& a, Vertex const& b, Vertex const& c) {
      trianglePart->vertices(row, 0) = a.pos.x;
      trianglePart->vertices(row, 1) = a.pos.y;
      trianglePart->vertices(row, 2) = a.pos.z;
      trianglePart->vertices(row, 3) = b.pos.x;
      trianglePart->vertices(row, 4) = b.pos.y;
      trianglePart->vertices(row, 5) = b.pos.z;
      trianglePart->vertices(row, 6) = c.pos.x;
      trianglePart->vertices(row, 7) = c.pos.y;
      trianglePart->vertices(row, 8) = c.pos.z;

      glm::dvec3 n = glm::cross(b.pos - a.pos, c.pos - a.pos);
      if (glm::length(n) > 0.0) {
        n = glm::normalize(n);
      }
      for (std::size_t j = 0; j < 3; ++j) {
        trianglePart->normals(row, j * 3) = n.x;
        trianglePart->normals(row, j * 3 + 1) = n.y;
        trianglePart->normals(row, j * 3 + 2) = n.z;
      }
    };

  std::size_t row = 0;
  for (int x = 0; x < rasterWidth - 1; ++x) {
    for (int y = 0; y < rasterHeight - 1; ++y) {
      Vertex* const vert0 = vertices[x][y];
      Vertex* const vert1 = vertices[x][y + 1];
      Vertex* const vert2 = vertices[x + 1][y + 1];
      Vertex* const vert3 = vertices[x + 1][y];

      if (hasTriangle(vert0, vert1, vert3)) {
        writeTriangle(row, *vert0, *vert1, *vert3);
        ++row;
      }

      if (hasTriangle(vert1, vert2, vert3)) {
        writeTriangle(row, *vert1, *vert2, *vert3);
        ++row;
      }
    }
  }
}
