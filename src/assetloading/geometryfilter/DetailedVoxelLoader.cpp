#include "DetailedVoxelLoader.h"
#include "VoxelFileParser.h"
#include <FileUtils.h>
#include <LadLutLoader.h>
#include <assetloading/MaterialsFileReader.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <noise/UniformNoiseSource.h>

#include <algorithm>
#include <stdexcept>

namespace fs = boost::filesystem;

ScenePart*
DetailedVoxelLoader::run()
{
  // Retrieve params
  bool transmittiveMode = false;
  if (params.find("intersectionMode") != params.end()) {
    primsOut->onRayIntersectionMode =
      boost::get<std::string>(params["intersectionMode"]);
    boost::to_upper(primsOut->onRayIntersectionMode);
    if (primsOut->onRayIntersectionMode == "SCALED") {
      primsOut->onRayIntersectionArgument = 0.5;
    }
  } else
    transmittiveMode = true;
  if (params.find("intersectionArgument") != params.end()) {
    primsOut->onRayIntersectionArgument =
      boost::get<double>(params["intersectionArgument"]);
  }
  if (params.find("randomShift") != params.end()) {
    primsOut->randomShift = boost::get<bool>(params["randomShift"]);
  }

  // Determine filepath
  std::vector<std::string> filePaths =
    FileUtils::handleFilePath(params, assetsDir);
  for (std::string filePath : filePaths) {
    std::stringstream ss;
    ss << "Reading detailed voxels from " << filePath;
    logging::INFO(ss.str());
  }

  // Load DV files
  for (std::string const& pathString : filePaths) {
    loadDv(pathString, transmittiveMode);
    primsOut->subpartLimit.push_back(primsOut->geometryCount());
  }

  // Load material if any
  loadMaterial();

  // Load ladlut if any
  loadLadlut();

  // Return detailed voxels as ScenePart *
  return primsOut;
}

void
DetailedVoxelLoader::loadDv(std::string const& pathString,
                            bool const discardNullPad)
{
  DetailedVoxelScenePart* const detailedPart =
    dynamic_cast<DetailedVoxelScenePart*>(primsOut);
  if (detailedPart == nullptr) {
    throw std::runtime_error(
      "DetailedVoxelLoader expected DetailedVoxelScenePart output");
  }

  // Check path exists
  fs::path fsPath(pathString);
  if (!fs::exists(pathString)) {
    std::stringstream ss;
    ss << "Voxel file not found: " << pathString;
    logging::ERR(ss.str());
    exit(1);
  }
  // Prepare default material
  Material mat;
  materials[mat.name] = std::make_shared<Material>(mat);
  // Legacy default material for vegetation studies commented below
  /*mat.isGround = false;
  mat.useVertexColors = true;
  mat.reflectance = 0.5;
  mat.specularity = 0.5;
  mat.classification = 1;
  mat.ka[0] = 0.5;    mat.ka[1] = 0.5;
  mat.ka[2] = 0.5;    mat.ka[2] = 0.5;
  mat.kd[0] = 0.5;    mat.kd[1] = 0.5;
  mat.kd[2] = 0.5;    mat.kd[3] = 0.5;
  mat.ks[0] = 0.5;    mat.ks[1] = 0.5;
  mat.ks[2] = 0.5;    mat.ks[3] = 0.5;
  mat.spectra = "wood";*/

  // Parse detailed voxels
  VoxelFileParser vfp;
  std::vector<DetailedVoxelRecord> dvs =
    vfp.parseDetailedRecords(pathString, 2, false, discardNullPad);

  if (dvs.empty()) {
    return;
  }

  std::size_t const oldRows = detailedPart->centers.n_rows;
  std::size_t const appendCount = dvs.size();
  std::size_t const newRows = oldRows + appendCount;

  std::size_t intCols = detailedPart->intData.n_cols;
  std::size_t doubleCols = detailedPart->doubleData.n_cols;
  for (DetailedVoxelRecord const& dv : dvs) {
    intCols = std::max(intCols, dv.intValues.size());
    doubleCols = std::max(doubleCols, dv.doubleValues.size());
  }

  auto resizeMatPreserve =
    [](arma::mat& mat, std::size_t rows, std::size_t cols) {
      arma::mat resized(rows, cols, arma::fill::zeros);
      if (mat.n_rows > 0 && mat.n_cols > 0) {
        resized.submat(0, 0, mat.n_rows - 1, mat.n_cols - 1) = mat;
      }
      mat = std::move(resized);
    };
  auto resizeIntMatPreserve =
    [](arma::Mat<int>& mat, std::size_t rows, std::size_t cols) {
      arma::Mat<int> resized(rows, cols, arma::fill::zeros);
      if (mat.n_rows > 0 && mat.n_cols > 0) {
        resized.submat(0, 0, mat.n_rows - 1, mat.n_cols - 1) = mat;
      }
      mat = std::move(resized);
    };
  auto resizeUVecPreserve = [](arma::uvec& vec, std::size_t rows) {
    arma::uvec resized(rows, arma::fill::zeros);
    if (vec.n_elem > 0) {
      resized.subvec(0, vec.n_elem - 1) = vec;
    }
    vec = std::move(resized);
  };

  resizeMatPreserve(detailedPart->centers, newRows, 3);
  resizeMatPreserve(detailedPart->halfSizes, newRows, 3);
  resizeMatPreserve(detailedPart->normals, newRows, 3);
  resizeUVecPreserve(detailedPart->materialIndex, newRows);
  resizeIntMatPreserve(detailedPart->intData, newRows, intCols);
  resizeMatPreserve(detailedPart->doubleData, newRows, doubleCols);
  detailedPart->identifiers.set_size(doubleCols);
  for (std::size_t i = 0; i < doubleCols; ++i) {
    detailedPart->identifiers(i) = i;
  }

  std::shared_ptr<Material> defaultMaterial = getMaterial(mat.name);
  for (std::size_t localIndex = 0; localIndex < appendCount; ++localIndex) {
    DetailedVoxelRecord const& dv = dvs[localIndex];
    std::size_t const row = oldRows + localIndex;

    detailedPart->centers(row, 0) = dv.center.x;
    detailedPart->centers(row, 1) = dv.center.y;
    detailedPart->centers(row, 2) = dv.center.z;
    detailedPart->normals(row, 0) = 0.0;
    detailedPart->normals(row, 1) = 0.0;
    detailedPart->normals(row, 2) = 0.0;
    detailedPart->halfSizes(row, 0) = dv.halfSize;
    detailedPart->halfSizes(row, 1) = dv.halfSize;
    detailedPart->halfSizes(row, 2) = dv.halfSize;

    std::size_t const iCols = dv.intValues.size();
    for (std::size_t col = 0; col < iCols; ++col) {
      detailedPart->intData(row, col) = dv.intValues[col];
    }
    std::size_t const dCols = dv.doubleValues.size();
    for (std::size_t col = 0; col < dCols; ++col) {
      detailedPart->doubleData(row, col) = dv.doubleValues[col];
    }
    detailedPart->maxPad = std::max(detailedPart->maxPad, dv.maxPad);

    if (defaultMaterial != nullptr) {
      detailedPart->setGeometryMaterial(row, defaultMaterial);
    }
  }
}

void
DetailedVoxelLoader::loadMaterial()
{
  // Parse materials
  std::vector<std::shared_ptr<Material>> matvec = parseMaterials();
  if (matvec.empty())
    return;

  // Assign material to each detailed voxel
  size_t j, n = primsOut->geometryCount(), m = matvec.size();
  for (size_t i = 0; i < n; i++) {
    j = i % m;
    primsOut->setGeometryMaterial(i, matvec[j]);
  }
}

void
DetailedVoxelLoader::loadLadlut()
{
  // If no LadLut is specified, get out of here
  if (params.find("ladlut") == params.end())
    return;

  // Load LadLut
  std::string ladlutPath = boost::get<std::string>(params["ladlut"]);
  LadLutLoader ladlutLoader;
  primsOut->ladlut = ladlutLoader.load(ladlutPath);
}
