#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <ScenePart.h>

#include <Material.h>

#include <cmath>
#include <memory>
#include <vector>

TEST_CASE("ScenePart: buildBulkFromPrimitives (triangles)")
{
  ScenePart sp;
  sp.primitiveType = ScenePart::PrimitiveType::TRIANGLE;

  auto mat0 = std::make_shared<Material>();
  auto mat1 = std::make_shared<Material>();

  Vertex a0(0.0, 0.0, 0.0);
  Vertex a1(1.0, 0.0, 0.0);
  Vertex a2(0.0, 1.0, 0.0);
  appendTriangleBulk(sp.triangles, a0, a1, a2, mat0);

  Vertex b0(0.0, 0.0, 1.0);
  Vertex b1(1.0, 0.0, 1.0);
  Vertex b2(0.0, 1.0, 1.0);
  appendTriangleBulk(sp.triangles, b0, b1, b2, mat1);

  REQUIRE(sp.triangles.size() == 2);
  REQUIRE(sp.triangles.vertices.size() == 6);
  REQUIRE(sp.triangles.materials.size() == 2);

  CHECK(sp.triangles.materials[0] == mat0);
  CHECK(sp.triangles.materials[1] == mat1);

  CHECK(sp.triangles.vertices[0].pos.x == 0.0);
  CHECK(sp.triangles.vertices[1].pos.x == 1.0);
  CHECK(sp.triangles.vertices[2].pos.y == 1.0);
  CHECK(sp.triangles.vertices[3].pos.z == 1.0);

  CHECK(sp.triangles.eps[0] == Catch::Approx(0.0000001));
  CHECK(sp.triangles.eps[1] == Catch::Approx(0.0000001));

  glm::dvec3 const n0 = sp.triangles.face_normal[0];
  CHECK(n0.x == Catch::Approx(0.0));
  CHECK(n0.y == Catch::Approx(0.0));
  CHECK(n0.z == Catch::Approx(1.0));

  CHECK(sp.triangles.aabb_min[0].x == Catch::Approx(0.0));
  CHECK(sp.triangles.aabb_min[0].y == Catch::Approx(0.0));
  CHECK(sp.triangles.aabb_min[0].z == Catch::Approx(0.0));
  CHECK(sp.triangles.aabb_max[0].x == Catch::Approx(1.0));
  CHECK(sp.triangles.aabb_max[0].y == Catch::Approx(1.0));
  CHECK(sp.triangles.aabb_max[0].z == Catch::Approx(0.0));
}

TEST_CASE("ScenePart: buildBulkFromPrimitives (voxels)")
{
  ScenePart sp;
  sp.primitiveType = ScenePart::PrimitiveType::VOXEL;

  auto mat0 = std::make_shared<Material>();
  auto mat1 = std::make_shared<Material>();

  appendVoxelBulk(sp.voxels,
                  Vertex(1.0, 2.0, 3.0),
                  1.0,
                  7,
                  0.1,
                  0.2,
                  0.3,
                  Color4f(0.4f, 0.5f, 0.6f, 0.7f),
                  mat0);

  std::vector<int> intValues = { 3, 4 };
  std::vector<double> doubleValues = { 1.5, 2.5, 3.5 };
  appendVoxelBulk(
    sp.voxels, Vertex(4.0, 5.0, 6.0), 1.5, 0, 0.0, 0.0, 0.0, Color4f(), mat1);
  sp.detailed_voxels.present.push_back(0);
  sp.detailed_voxels.int_values.emplace_back();
  sp.detailed_voxels.double_values.emplace_back();
  sp.detailed_voxels.max_pad.push_back(0.0);
  sp.detailed_voxels.present.push_back(1);
  sp.detailed_voxels.int_values.push_back(intValues);
  sp.detailed_voxels.double_values.push_back(doubleValues);
  sp.detailed_voxels.max_pad.push_back(9.0);

  REQUIRE(sp.voxels.size() == 2);
  REQUIRE(sp.detailed_voxels.size() == 2);

  CHECK(sp.voxels.centers[0].pos.x == Catch::Approx(1.0));
  CHECK(sp.voxels.centers[0].pos.y == Catch::Approx(2.0));
  CHECK(sp.voxels.centers[0].pos.z == Catch::Approx(3.0));
  CHECK(sp.voxels.half_size[0] == Catch::Approx(1.0));
  CHECK(sp.voxels.num_points[0] == 7);
  CHECK(sp.voxels.r[0] == Catch::Approx(0.1));
  CHECK(sp.voxels.g[0] == Catch::Approx(0.2));
  CHECK(sp.voxels.b[0] == Catch::Approx(0.3));
  CHECK(sp.voxels.materials[0] == mat0);

  CHECK(sp.voxels.aabb_min[0].x == Catch::Approx(0.0));
  CHECK(sp.voxels.aabb_min[0].y == Catch::Approx(1.0));
  CHECK(sp.voxels.aabb_min[0].z == Catch::Approx(2.0));
  CHECK(sp.voxels.aabb_max[0].x == Catch::Approx(2.0));
  CHECK(sp.voxels.aabb_max[0].y == Catch::Approx(3.0));
  CHECK(sp.voxels.aabb_max[0].z == Catch::Approx(4.0));

  CHECK(sp.detailed_voxels.present[0] == 0);
  CHECK(sp.detailed_voxels.present[1] == 1);

  REQUIRE(sp.detailed_voxels.int_values.size() == 2);
  REQUIRE(sp.detailed_voxels.double_values.size() == 2);
  CHECK(sp.detailed_voxels.int_values[1].size() == 2);
  CHECK(sp.detailed_voxels.int_values[1][0] == 3);
  CHECK(sp.detailed_voxels.double_values[1].size() == 3);
  CHECK(sp.detailed_voxels.double_values[1][2] == Catch::Approx(3.5));
  CHECK(sp.detailed_voxels.max_pad[1] == Catch::Approx(9.0));
  CHECK(sp.voxels.materials[1] == mat1);
}
