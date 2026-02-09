#include <catch2/catch_test_macros.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <DetailedVoxel.h>
#include <Scene.h>
#include <SerialIO.h>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// validation functions
void
validate(DetailedVoxel& dv1, DetailedVoxel& dv2)
{
  glm::dvec3 dv1c = dv1.getCentroid();
  glm::dvec3 dv2c = dv2.getCentroid();
  REQUIRE(dv1c.x == dv2c.x);
  REQUIRE(dv1c.y == dv2c.y);
  REQUIRE(dv1c.z == dv2c.z);
  REQUIRE(dv1.halfSize == dv2.halfSize);
  for (size_t i = 0; i < dv1.getNumberOfIntValues(); i++) {
    REQUIRE(dv1.getIntValue(i) == dv2.getIntValue(i));
  }
  for (size_t i = 0; i < dv1.getNumberOfDoubleValues(); i++) {
    REQUIRE(dv1[i] == dv2[i]);
  }
  if (dv1.material != nullptr)
    REQUIRE(dv2.material != nullptr);
  if (dv2.material != nullptr)
    REQUIRE(dv1.material != nullptr);
  if (dv1.material != nullptr && dv2.material != nullptr) {
    for (size_t i = 0; i < 4; i++) {
      REQUIRE(dv1.material->ka[i] == dv2.material->ka[i]);
    }
  }
}

void
validate(Voxel& v1, Voxel& v2)
{
  glm::dvec3 v1c = v1.getCentroid();
  glm::dvec3 v2c = v2.getCentroid();
  REQUIRE(v1c.x == v2c.x);
  REQUIRE(v1c.y == v2c.y);
  REQUIRE(v1c.z == v2c.z);
  REQUIRE(v1.halfSize == v2.halfSize);
}

void
validate(AABB& box1, AABB& box2)
{
  double minX1 = box1.getMin().x;
  double maxX1 = box1.getMax().x;
  double minY1 = box1.getMin().y;
  double maxY1 = box1.getMax().y;
  double minZ1 = box1.getMin().z;
  double maxZ1 = box1.getMax().z;
  double minX2 = box2.getMin().x;
  double maxX2 = box2.getMax().x;
  double minY2 = box2.getMin().y;
  double maxY2 = box2.getMax().y;
  double minZ2 = box2.getMin().z;
  double maxZ2 = box2.getMax().z;

  REQUIRE(minX1 == minX2);
  REQUIRE(minY1 == minY2);
  REQUIRE(minZ1 == minZ2);
  REQUIRE(maxX1 == maxX2);
  REQUIRE(maxY1 == maxY2);
  REQUIRE(maxZ1 == maxZ2);
}

void
validate(Triangle& t1, Triangle& t2)
{
  double x1, x2, y1, y2, z1, z2;
  for (size_t i = 0; i < t1.getNumVertices(); i++) {
    x1 = t1.verts[i].getX();
    x2 = t2.verts[i].getX();
    REQUIRE(x1 == x2);
    y1 = t1.verts[i].getY();
    y2 = t2.verts[i].getY();
    REQUIRE(y1 == y2);
    z1 = t1.verts[i].getZ();
    z2 = t2.verts[i].getZ();
    REQUIRE(z1 == z2);
  }
}

TEST_CASE("Serialization Test")
{
  std::string path = "SerializationTest.tmp";
  DetailedVoxel dv1(1.0,
                    2.0,
                    1.0,
                    1.0,
                    std::vector<int>({ 0, 1, 2 }),
                    std::vector<double>({ 1.0, 1.5, 2.0, 2.5, 3.0 }));
  dv1.material = std::make_shared<Material>();
  dv1.material->ka[0] = 1.0;
  dv1.material->ka[1] = 2.0;
  dv1.material->ka[2] = 3.0;
  dv1.material->ka[3] = 4.0;

  SECTION("DetailedVoxel Serialization")
  {
    SerialIO* sio = SerialIO::getInstance();
    sio->write<DetailedVoxel>(path, &dv1);
    DetailedVoxel* dv2 = sio->read<DetailedVoxel>(path);

    // validate detailed voxel
    validate(dv1, *dv2);
    delete dv2;
  }

  SECTION("Scene Serialization")
  {
    std::size_t nRepeats = 32;
    Scene scene1;
    Vertex t1v1, t1v2, t1v3;
    t1v1.pos = glm::dvec3(0.0, 0.0, 0.0);
    t1v2.pos = glm::dvec3(0.0, 0.0, 4.0);
    t1v3.pos = glm::dvec3(2.0, 0.0, 0.0);
    std::shared_ptr<ScenePart> triPart = std::make_shared<ScenePart>();
    triPart->primitiveType = ScenePart::PrimitiveType::TRIANGLE;
    auto triMat = std::make_shared<Material>();
    appendTriangleBulk(triPart->triangles, t1v1, t1v2, t1v3, triMat);

    std::shared_ptr<ScenePart> voxelPart = std::make_shared<ScenePart>();
    voxelPart->primitiveType = ScenePart::PrimitiveType::VOXEL;
    voxelPart->onRayIntersectionMode = "TRANSMITTIVE";
    auto dvMat = std::make_shared<Material>();
    dvMat->ka[0] = 1.0;
    dvMat->ka[1] = 2.0;
    dvMat->ka[2] = 3.0;
    dvMat->ka[3] = 4.0;
    auto vMat = std::make_shared<Material>();
    vMat->ks[0] = 0.4;
    vMat->ks[1] = 0.6;

    auto appendDetailed = [&](Vertex const& center,
                              double halfSize,
                              std::shared_ptr<Material> const& mat,
                              bool present,
                              std::vector<int> const& intValues,
                              std::vector<double> const& doubleValues,
                              double maxPad) {
      appendVoxelBulk(
        voxelPart->voxels, center, halfSize, 0, 0.0, 0.0, 0.0, Color4f(), mat);
      voxelPart->detailed_voxels.present.push_back(present ? 1 : 0);
      if (present) {
        voxelPart->detailed_voxels.int_values.push_back(intValues);
        voxelPart->detailed_voxels.double_values.push_back(doubleValues);
        voxelPart->detailed_voxels.max_pad.push_back(maxPad);
      } else {
        voxelPart->detailed_voxels.int_values.emplace_back();
        voxelPart->detailed_voxels.double_values.emplace_back();
        voxelPart->detailed_voxels.max_pad.push_back(0.0);
      }
    };

    appendDetailed(Vertex(1.0, 2.0, 1.0),
                   1.0,
                   dvMat,
                   true,
                   { 0, 1, 2 },
                   { 1.0, 1.5, 2.0, 2.5, 3.0 },
                   0.0);
    appendDetailed(Vertex(1.0, 1.0, 1.0), 1.0, vMat, false, {}, {}, 0.0);
    for (std::size_t i = 0; i < nRepeats; i++) {
      appendDetailed(Vertex(1.0, 2.0, 1.0),
                     1.0,
                     dvMat,
                     true,
                     { 0, 1, 2 },
                     { 1.0, 1.5, 2.0, 2.5, 3.0 },
                     0.0);
    }

    scene1.parts.push_back(triPart);
    scene1.parts.push_back(voxelPart);
    scene1.finalizeLoading(true);
    std::shared_ptr<KDGroveFactory> kdgf = scene1.getKDGroveFactory();
    scene1.writeObject(path);
    scene1.setKDGroveFactory(kdgf);
    Scene* scene2 = Scene::readObject(path);

    REQUIRE(scene2->parts.size() == scene1.parts.size());
    REQUIRE(scene2->parts.size() == 2);

    auto& triPart1 = *scene1.parts[0];
    auto& triPart2 = *scene2->parts[0];
    REQUIRE(triPart1.primitiveType == triPart2.primitiveType);
    REQUIRE(triPart1.triangles.size() == triPart2.triangles.size());
    REQUIRE(triPart1.triangles.vertices.size() ==
            triPart2.triangles.vertices.size());
    for (size_t i = 0; i < triPart1.triangles.vertices.size(); ++i) {
      REQUIRE(triPart1.triangles.vertices[i].pos.x ==
              triPart2.triangles.vertices[i].pos.x);
      REQUIRE(triPart1.triangles.vertices[i].pos.y ==
              triPart2.triangles.vertices[i].pos.y);
      REQUIRE(triPart1.triangles.vertices[i].pos.z ==
              triPart2.triangles.vertices[i].pos.z);
    }

    auto& voxelPart1 = *scene1.parts[1];
    auto& voxelPart2 = *scene2->parts[1];
    REQUIRE(voxelPart1.primitiveType == voxelPart2.primitiveType);
    REQUIRE(voxelPart1.voxels.size() == voxelPart2.voxels.size());
    REQUIRE(voxelPart1.detailed_voxels.size() ==
            voxelPart2.detailed_voxels.size());
    for (size_t i = 0; i < voxelPart1.voxels.size(); ++i) {
      REQUIRE(voxelPart1.voxels.centers[i].pos.x ==
              voxelPart2.voxels.centers[i].pos.x);
      REQUIRE(voxelPart1.voxels.centers[i].pos.y ==
              voxelPart2.voxels.centers[i].pos.y);
      REQUIRE(voxelPart1.voxels.centers[i].pos.z ==
              voxelPart2.voxels.centers[i].pos.z);
      REQUIRE(voxelPart1.voxels.half_size[i] == voxelPart2.voxels.half_size[i]);
      REQUIRE(voxelPart1.detailed_voxels.present[i] ==
              voxelPart2.detailed_voxels.present[i]);
      REQUIRE(voxelPart1.detailed_voxels.int_values[i].size() ==
              voxelPart2.detailed_voxels.int_values[i].size());
      REQUIRE(voxelPart1.detailed_voxels.double_values[i].size() ==
              voxelPart2.detailed_voxels.double_values[i].size());
      for (size_t j = 0; j < voxelPart1.detailed_voxels.int_values[i].size();
           ++j) {
        REQUIRE(voxelPart1.detailed_voxels.int_values[i][j] ==
                voxelPart2.detailed_voxels.int_values[i][j]);
      }
      for (size_t j = 0; j < voxelPart1.detailed_voxels.double_values[i].size();
           ++j) {
        REQUIRE(voxelPart1.detailed_voxels.double_values[i][j] ==
                voxelPart2.detailed_voxels.double_values[i][j]);
      }
      REQUIRE(voxelPart1.detailed_voxels.max_pad[i] ==
              voxelPart2.detailed_voxels.max_pad[i]);
      if (voxelPart1.voxels.materials[i] != nullptr) {
        REQUIRE(voxelPart2.voxels.materials[i] != nullptr);
        REQUIRE(voxelPart1.voxels.materials[i]->ka[0] ==
                voxelPart2.voxels.materials[i]->ka[0]);
        REQUIRE(voxelPart1.voxels.materials[i]->ka[1] ==
                voxelPart2.voxels.materials[i]->ka[1]);
        REQUIRE(voxelPart1.voxels.materials[i]->ka[2] ==
                voxelPart2.voxels.materials[i]->ka[2]);
        REQUIRE(voxelPart1.voxels.materials[i]->ka[3] ==
                voxelPart2.voxels.materials[i]->ka[3]);
        REQUIRE(voxelPart1.voxels.materials[i]->ks[0] ==
                voxelPart2.voxels.materials[i]->ks[0]);
        REQUIRE(voxelPart1.voxels.materials[i]->ks[1] ==
                voxelPart2.voxels.materials[i]->ks[1]);
      } else {
        REQUIRE(voxelPart2.voxels.materials[i] == nullptr);
      }
    }

    REQUIRE(scene1.getAABB() != nullptr);
    REQUIRE(scene2->getAABB() != nullptr);
    validate(*scene1.getAABB(), *scene2->getAABB());
    REQUIRE(scene1.getBBoxCRS() != nullptr);
    REQUIRE(scene2->getBBoxCRS() != nullptr);
    validate(*scene1.getBBoxCRS(), *scene2->getBBoxCRS());

    delete scene2;
  }

  // Cleanup
  std::remove(path.c_str());
}
