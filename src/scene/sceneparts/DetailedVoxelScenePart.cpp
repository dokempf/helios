#include <DetailedVoxelScenePart.h>

#include <IntersectionHandlingResult.h>
#include <NoiseSource.h>

#include <algorithm>
#include <cmath>

DetailedVoxelScenePart::DetailedVoxelScenePart()
  : VoxelScenePart()
{
  geometryType = GeometryType::DETAILED_VOXEL;
}

DetailedVoxelScenePart::DetailedVoxelScenePart(std::size_t voxelCount)
  : VoxelScenePart(voxelCount)
{
  geometryType = GeometryType::DETAILED_VOXEL;
}

std::shared_ptr<ScenePart>
DetailedVoxelScenePart::clone(bool) const
{
  std::shared_ptr<DetailedVoxelScenePart> out =
    std::make_shared<DetailedVoxelScenePart>(*this);
  out->materialTable.clear();
  out->materialTable.reserve(materialTable.size());
  for (std::shared_ptr<Material> const& material : materialTable) {
    if (material == nullptr) {
      out->materialTable.push_back(nullptr);
    } else {
      out->materialTable.push_back(std::make_shared<Material>(*material));
    }
  }
  return out;
}

ScenePart::GeometryType
DetailedVoxelScenePart::geometryTypeOf() const
{
  return GeometryType::DETAILED_VOXEL;
}

bool
DetailedVoxelScenePart::geometryCanComputeSigmaWithLadLut(
  std::size_t index) const
{
  return index < geometryCount() && ladlut != nullptr;
}

double
DetailedVoxelScenePart::geometryComputeSigmaWithLadLut(
  std::size_t index,
  glm::dvec3 const& direction) const
{
  if (!geometryCanComputeSigmaWithLadLut(index) || doubleData.n_cols == 0) {
    return 0.0;
  }
  double const sigma = doubleData(index, 0); // PadBVTotal
  return ladlut->computeSigma(sigma, direction.x, direction.y, direction.z);
}

bool
DetailedVoxelScenePart::geometryCanHandleIntersections(std::size_t index) const
{
  return index < geometryCount();
}

IntersectionHandlingResult
DetailedVoxelScenePart::geometryOnRayIntersection(
  std::size_t index,
  NoiseSource<double>& uniformNoiseSource,
  glm::dvec3& rayDirection,
  glm::dvec3 const& insideIntersectionPoint,
  glm::dvec3 const& outsideIntersectionPoint,
  double) const
{
  if (!geometryCanHandleIntersections(index) || doubleData.n_cols == 0) {
    return {};
  }

  std::string const& mode = onRayIntersectionMode;
  if (mode == "FIXED" || mode == "SCALED") {
    if (doubleData.n_cols <= 8) {
      return IntersectionHandlingResult(insideIntersectionPoint, true);
    }
    double const transmittance = doubleData(index, 8);
    return IntersectionHandlingResult(insideIntersectionPoint,
                                      transmittance == 1.0 ||
                                        std::isnan(transmittance));
  }

  // Default: TRANSMITTIVE
  double intersectionLength =
    glm::distance(insideIntersectionPoint, outsideIntersectionPoint);
  double sigma = doubleData(index, 0); // PadBVTotal
  if (ladlut != nullptr) {
    sigma = ladlut->computeSigma(
      sigma, rayDirection.x, rayDirection.y, rayDirection.z);
  }

  if (std::isnan(sigma) || sigma == 0.0) {
    return IntersectionHandlingResult(insideIntersectionPoint, true);
  }

  double const rndProb = uniformNoiseSource.next();
  double const s = -std::log(rndProb) / sigma;
  if (s > intersectionLength) {
    return IntersectionHandlingResult(insideIntersectionPoint, true);
  }
  glm::dvec3 const nip = insideIntersectionPoint + rayDirection * s;
  return IntersectionHandlingResult(nip, false);
}

void
DetailedVoxelScenePart::geometryOnFinishLoading(
  std::size_t index,
  NoiseSource<double>& uniformNoiseSource)
{
  if (index >= geometryCount() || onRayIntersectionMode != "SCALED" ||
      doubleData.n_cols == 0) {
    return;
  }

  double currentMaxPad = maxPad;
  if (currentMaxPad == 0.0 && doubleData.n_rows > 0) {
    for (std::size_t i = 0; i < doubleData.n_rows; ++i) {
      currentMaxPad = std::max(currentMaxPad, doubleData(i, 0));
    }
  }
  if (currentMaxPad == 0.0) {
    return;
  }

  double const pad = doubleData(index, 0);
  double const oldHalfSize = halfSizes(index, 0);
  double newHalfSize =
    oldHalfSize * std::pow(pad / currentMaxPad, onRayIntersectionArgument);

  if (randomShift) {
    double maxShift = oldHalfSize - newHalfSize;
    if (std::isnan(maxShift)) {
      maxShift = 0.0;
    }
    centers(index, 0) += uniformNoiseSource.next() * maxShift;
    centers(index, 1) += uniformNoiseSource.next() * maxShift;
    centers(index, 2) += uniformNoiseSource.next() * maxShift;
  }

  halfSizes(index, 0) = newHalfSize;
  halfSizes(index, 1) = newHalfSize;
  halfSizes(index, 2) = newHalfSize;
  geometryUpdate(index);
}
