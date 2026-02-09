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
  if (primitiveType == PrimitiveType::TRIANGLE)
    return triangles.vertices.size();
  if (primitiveType == PrimitiveType::VOXEL)
    return voxels.centers.size();

  return 0;
}

arma::mat
DynObject::matrixFromPrimitives(
  std::function<arma::colvec(Vertex const*)> get) const
{
  return matrixFromPrimitives(countVertices(), get);
}
arma::mat
DynObject::matrixFromPrimitives(
  size_t const m,
  std::function<arma::colvec(Vertex const*)> get) const
{
  arma::mat X(3, m);
  size_t i = 0;
  if (primitiveType == PrimitiveType::TRIANGLE) {
    for (Vertex const& v : triangles.vertices) {
      X.col(i++) = get(&v);
    }
    return X;
  }
  if (primitiveType == PrimitiveType::VOXEL) {
    for (Vertex const& v : voxels.centers) {
      X.col(i++) = get(&v);
    }
    return X;
  }
  return X;
}

void
DynObject::matrixToPrimitives(
  std::function<void(Vertex*, arma::colvec const&)> set,
  arma::mat const& X)
{
  matrixToPrimitives(countVertices(), set, X);
}
void
DynObject::matrixToPrimitives(
  size_t const m,
  std::function<void(Vertex*, arma::colvec const&)> set,
  arma::mat const& X)
{
  size_t i = 0;
  if (primitiveType == PrimitiveType::TRIANGLE) {
    for (Vertex& v : triangles.vertices) {
      set(&v, X.col(i++));
    }
    return;
  }
  if (primitiveType == PrimitiveType::VOXEL) {
    for (Vertex& v : voxels.centers) {
      set(&v, X.col(i++));
    }
    return;
  }
}

arma::mat
DynObject::positionMatrixFromPrimitives() const
{
  return positionMatrixFromPrimitives(countVertices());
}
arma::mat
DynObject::positionMatrixFromPrimitives(size_t const m) const
{
  return matrixFromPrimitives(m, [](Vertex const* p) -> arma::colvec {
    arma::colvec x(3);
    x(0) = p->getX();
    x(1) = p->getY();
    x(2) = p->getZ();
    return x;
  });
}

arma::mat
DynObject::normalMatrixFromPrimitives() const
{
  return normalMatrixFromPrimitives(countVertices());
}
arma::mat
DynObject::normalMatrixFromPrimitives(size_t const m) const
{
  return matrixFromPrimitives(m, [](Vertex const* p) -> arma::colvec {
    arma::colvec x(3);
    x(0) = p->normal.x;
    x(1) = p->normal.y;
    x(2) = p->normal.z;
    return x;
  });
}

void
DynObject::updatePrimitivesPositionFromMatrix(arma::mat const& X)
{
  updatePrimitivesPositionFromMatrix(countVertices(), X);
}
void
DynObject::updatePrimitivesPositionFromMatrix(size_t const m,
                                              arma::mat const& X)
{
  matrixToPrimitives(
    m,
    [](Vertex* p, arma::colvec const& x) -> void {
      p->pos.x = x(0);
      p->pos.y = x(1);
      p->pos.z = x(2);
    },
    X);
}

void
DynObject::updatePrimitivesNormalFromMatrix(arma::mat const& X)
{
  updatePrimitivesNormalFromMatrix(countVertices(), X);
}
void
DynObject::updatePrimitivesNormalFromMatrix(size_t const m, arma::mat const& X)
{
  matrixToPrimitives(
    m,
    [](Vertex* p, arma::colvec const& x) -> void {
      p->normal.x = x(0);
      p->normal.y = x(1);
      p->normal.z = x(2);
    },
    X);
}
