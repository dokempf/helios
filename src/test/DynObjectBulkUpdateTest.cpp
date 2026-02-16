#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <scene/dynamic/DynObject.h>

namespace {
class BulkDynObject : public DynObject
{
public:
  arma::mat positions;
  arma::mat normals;

  BulkDynObject()
  {
    positions.zeros(1, 6);
    normals.zeros(1, 6);
  }

  bool doSimStep() override { return false; }

  std::size_t geometryCount() const override { return positions.n_rows; }

  std::size_t geometryDynamicVertexCount(
    std::size_t geometryIndex) const override
  {
    return geometryIndex < positions.n_rows ? 2 : 0;
  }

  std::size_t geometryVertexCount(std::size_t geometryIndex) const override
  {
    return geometryDynamicVertexCount(geometryIndex);
  }

  glm::dvec3 geometryDynamicVertexPosition(
    std::size_t geometryIndex,
    std::size_t vertexIndex) const override
  {
    if (geometryIndex >= positions.n_rows || vertexIndex >= 2) {
      return glm::dvec3(0.0);
    }
    std::size_t const base = 3 * vertexIndex;
    return glm::dvec3(positions(geometryIndex, base),
                      positions(geometryIndex, base + 1),
                      positions(geometryIndex, base + 2));
  }

  glm::dvec3 geometryDynamicVertexNormal(std::size_t geometryIndex,
                                         std::size_t vertexIndex) const override
  {
    if (geometryIndex >= normals.n_rows || vertexIndex >= 2) {
      return glm::dvec3(0.0);
    }
    std::size_t const base = 3 * vertexIndex;
    return glm::dvec3(normals(geometryIndex, base),
                      normals(geometryIndex, base + 1),
                      normals(geometryIndex, base + 2));
  }

  void setGeometryDynamicVertexPosition(std::size_t geometryIndex,
                                        std::size_t vertexIndex,
                                        glm::dvec3 const& p) override
  {
    if (geometryIndex >= positions.n_rows || vertexIndex >= 2) {
      return;
    }
    std::size_t const base = 3 * vertexIndex;
    positions(geometryIndex, base) = p.x;
    positions(geometryIndex, base + 1) = p.y;
    positions(geometryIndex, base + 2) = p.z;
  }

  void setGeometryDynamicVertexNormal(std::size_t geometryIndex,
                                      std::size_t vertexIndex,
                                      glm::dvec3 const& n) override
  {
    if (geometryIndex >= normals.n_rows || vertexIndex >= 2) {
      return;
    }
    std::size_t const base = 3 * vertexIndex;
    normals(geometryIndex, base) = n.x;
    normals(geometryIndex, base + 1) = n.y;
    normals(geometryIndex, base + 2) = n.z;
  }

  void geometryUpdate(std::size_t) override {}
};
}

TEST_CASE("DynObject: Bulk matrix extraction and update")
{
  BulkDynObject obj;
  obj.positions.row(0) = arma::rowvec({ 0.0, 0.0, 0.0, 1.0, 0.0, 0.0 });
  obj.normals.row(0) = arma::rowvec({ 0.0, 0.0, 1.0, 0.0, 1.0, 0.0 });

  REQUIRE(obj.countVertices() == 2);

  arma::mat P = obj.positionMatrixFromGeometry();
  REQUIRE(P.n_rows == 3);
  REQUIRE(P.n_cols == 2);
  REQUIRE(P(0, 0) == Catch::Approx(0.0));
  REQUIRE(P(0, 1) == Catch::Approx(1.0));

  P(0, 0) = 10.0;
  P(1, 1) = 20.0;
  obj.updateGeometryPositionFromMatrix(P);
  REQUIRE(obj.geometryDynamicVertexPosition(0, 0).x == Catch::Approx(10.0));
  REQUIRE(obj.geometryDynamicVertexPosition(0, 1).y == Catch::Approx(20.0));

  arma::mat N = obj.normalMatrixFromGeometry(2);
  REQUIRE(N.n_rows == 3);
  REQUIRE(N.n_cols == 2);
  N(2, 0) = -1.0;
  N(2, 1) = -2.0;
  obj.updateGeometryNormalFromMatrix(2, N);
  REQUIRE(obj.geometryDynamicVertexNormal(0, 0).z == Catch::Approx(-1.0));
  REQUIRE(obj.geometryDynamicVertexNormal(0, 1).z == Catch::Approx(-2.0));
}
