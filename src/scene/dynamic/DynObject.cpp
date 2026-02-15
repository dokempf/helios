#include <scene/dynamic/DynObject.h>

// ***  DYNAMIC BEHAVIOR  *** //
// ************************** //
bool
DynObject::doStep()
{
  if (stepLoop.doStep())
    return stepLoop.retrieveOutput();
  return false;
}

// ***  U T I L  *** //
// ***************** //
size_t
DynObject::countVertices() const
{
  size_t m = 0;
  for (size_t i = 0; i < geometryCount(); ++i) {
    m += geometryDynamicVertexCount(i);
  }
  return m;
}

arma::mat
DynObject::matrixFromGeometry(
  std::function<arma::colvec(std::size_t, std::size_t)> get) const
{
  return matrixFromGeometry(countVertices(), get);
}
arma::mat
DynObject::matrixFromGeometry(
  size_t const m,
  std::function<arma::colvec(std::size_t, std::size_t)> get) const
{
  arma::mat X(3, m);
  size_t i = 0;
  for (size_t j = 0; j < geometryCount(); ++j) {
    std::size_t const n = geometryDynamicVertexCount(j);
    for (size_t k = 0; k < n; ++k, ++i) {
      X.col(i) = get(j, k);
    }
  }
  return X;
}

void
DynObject::matrixToGeometry(
  std::function<void(std::size_t, std::size_t, arma::colvec const&)> set,
  arma::mat const& X)
{
  matrixToGeometry(countVertices(), set, X);
}
void
DynObject::matrixToGeometry(
  size_t const m,
  std::function<void(std::size_t, std::size_t, arma::colvec const&)> set,
  arma::mat const& X)
{
  size_t i = 0;
  for (size_t j = 0; j < geometryCount(); ++j) {
    std::size_t const n = geometryDynamicVertexCount(j);
    for (size_t k = 0; k < n; ++k, ++i) {
      set(j, k, X.col(i));
    }
  }
}

arma::mat
DynObject::positionMatrixFromGeometry() const
{
  return positionMatrixFromGeometry(countVertices());
}
arma::mat
DynObject::positionMatrixFromGeometry(size_t const m) const
{
  return matrixFromGeometry(
    m,
    [this](std::size_t geometryIndex, std::size_t vertexIndex) -> arma::colvec {
      glm::dvec3 const p =
        geometryDynamicVertexPosition(geometryIndex, vertexIndex);
      arma::colvec x(3);
      x(0) = p.x;
      x(1) = p.y;
      x(2) = p.z;
      return x;
    });
}

arma::mat
DynObject::normalMatrixFromGeometry() const
{
  return normalMatrixFromGeometry(countVertices());
}
arma::mat
DynObject::normalMatrixFromGeometry(size_t const m) const
{
  return matrixFromGeometry(
    m,
    [this](std::size_t geometryIndex, std::size_t vertexIndex) -> arma::colvec {
      glm::dvec3 const n =
        geometryDynamicVertexNormal(geometryIndex, vertexIndex);
      arma::colvec x(3);
      x(0) = n.x;
      x(1) = n.y;
      x(2) = n.z;
      return x;
    });
}

void
DynObject::updateGeometryPositionFromMatrix(arma::mat const& X)
{
  updateGeometryPositionFromMatrix(countVertices(), X);
}
void
DynObject::updateGeometryPositionFromMatrix(size_t const m, arma::mat const& X)
{
  matrixToGeometry(
    m,
    [this](std::size_t geometryIndex,
           std::size_t vertexIndex,
           arma::colvec const& x) -> void {
      setGeometryDynamicVertexPosition(
        geometryIndex, vertexIndex, glm::dvec3(x(0), x(1), x(2)));
    },
    X);
}

void
DynObject::updateGeometryNormalFromMatrix(arma::mat const& X)
{
  updateGeometryNormalFromMatrix(countVertices(), X);
}
void
DynObject::updateGeometryNormalFromMatrix(size_t const m, arma::mat const& X)
{
  matrixToGeometry(
    m,
    [this](std::size_t geometryIndex,
           std::size_t vertexIndex,
           arma::colvec const& x) -> void {
      setGeometryDynamicVertexNormal(
        geometryIndex, vertexIndex, glm::dvec3(x(0), x(1), x(2)));
    },
    X);
}
