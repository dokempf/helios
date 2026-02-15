#include <catch2/catch_test_macros.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <DetailedVoxelScenePart.h>
#include <Scene.h>
#include <SerialIO.h>
#include <TriangleScenePart.h>
#include <VoxelScenePart.h>

#include <cstdio>
#include <string>
#include <vector>

namespace {
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
validate(DetailedVoxelScenePart const& p1, DetailedVoxelScenePart const& p2)
{
  REQUIRE(p1.geometryCount() == p2.geometryCount());
  REQUIRE(p1.intData.n_rows == p2.intData.n_rows);
  REQUIRE(p1.intData.n_cols == p2.intData.n_cols);
  REQUIRE(p1.doubleData.n_rows == p2.doubleData.n_rows);
  REQUIRE(p1.doubleData.n_cols == p2.doubleData.n_cols);
  REQUIRE(p1.identifiers.n_elem == p2.identifiers.n_elem);
  REQUIRE(p1.maxPad == p2.maxPad);

  for (std::size_t i = 0; i < p1.geometryCount(); ++i) {
    glm::dvec3 const c1 = p1.geometryCentroid(i);
    glm::dvec3 const c2 = p2.geometryCentroid(i);
    REQUIRE(c1.x == c2.x);
    REQUIRE(c1.y == c2.y);
    REQUIRE(c1.z == c2.z);
    REQUIRE(p1.halfSizes(i, 0) == p2.halfSizes(i, 0));
    REQUIRE(p1.halfSizes(i, 1) == p2.halfSizes(i, 1));
    REQUIRE(p1.halfSizes(i, 2) == p2.halfSizes(i, 2));
  }

  for (std::size_t r = 0; r < p1.intData.n_rows; ++r) {
    for (std::size_t c = 0; c < p1.intData.n_cols; ++c) {
      REQUIRE(p1.intData(r, c) == p2.intData(r, c));
    }
  }

  for (std::size_t r = 0; r < p1.doubleData.n_rows; ++r) {
    for (std::size_t c = 0; c < p1.doubleData.n_cols; ++c) {
      REQUIRE(p1.doubleData(r, c) == p2.doubleData(r, c));
    }
  }

  for (std::size_t i = 0; i < p1.identifiers.n_elem; ++i) {
    REQUIRE(p1.identifiers(i) == p2.identifiers(i));
  }

  REQUIRE(p1.materialTable.size() == p2.materialTable.size());
  if (!p1.materialTable.empty() && p1.materialTable[0] != nullptr &&
      p2.materialTable[0] != nullptr) {
    for (std::size_t i = 0; i < 4; ++i) {
      REQUIRE(p1.materialTable[0]->ka[i] == p2.materialTable[0]->ka[i]);
    }
  }
}
}

TEST_CASE("Serialization Test")
{
  std::string path = "SerializationTest.tmp";

  auto dvPart = std::make_shared<DetailedVoxelScenePart>(1);
  dvPart->centers.zeros(1, 3);
  dvPart->halfSizes.zeros(1, 3);
  dvPart->normals.zeros(1, 3);
  dvPart->intData.zeros(1, 3);
  dvPart->doubleData.zeros(1, 5);
  dvPart->identifiers.zeros(5);
  dvPart->materialIndex.zeros(1);
  dvPart->centers(0, 0) = 1.0;
  dvPart->centers(0, 1) = 2.0;
  dvPart->centers(0, 2) = 1.0;
  dvPart->halfSizes(0, 0) = 1.0;
  dvPart->halfSizes(0, 1) = 1.0;
  dvPart->halfSizes(0, 2) = 1.0;
  dvPart->intData(0, 0) = 0;
  dvPart->intData(0, 1) = 1;
  dvPart->intData(0, 2) = 2;
  dvPart->doubleData(0, 0) = 1.0;
  dvPart->doubleData(0, 1) = 1.5;
  dvPart->doubleData(0, 2) = 2.0;
  dvPart->doubleData(0, 3) = 2.5;
  dvPart->doubleData(0, 4) = 3.0;
  dvPart->maxPad = 3.0;
  for (std::size_t i = 0; i < dvPart->identifiers.n_elem; ++i) {
    dvPart->identifiers(i) = i;
  }
  dvPart->onRayIntersectionMode = "TRANSMITTIVE";
  dvPart->materialTable.push_back(std::make_shared<Material>());
  dvPart->materialTable[0]->ka[0] = 1.0;
  dvPart->materialTable[0]->ka[1] = 2.0;
  dvPart->materialTable[0]->ka[2] = 3.0;
  dvPart->materialTable[0]->ka[3] = 4.0;
  dvPart->materialIndex(0) = 0;

  SECTION("DetailedVoxelScenePart Serialization")
  {
    SerialIO* sio = SerialIO::getInstance();
    sio->write<DetailedVoxelScenePart>(path, dvPart.get());
    DetailedVoxelScenePart* dvPart2 = sio->read<DetailedVoxelScenePart>(path);
    REQUIRE(dvPart2 != nullptr);
    validate(*dvPart, *dvPart2);
    delete dvPart2;
  }

  SECTION("Scene Serialization")
  {
    std::size_t const nRepeats = 32;
    Scene scene1;

    auto detailedPart = std::make_shared<DetailedVoxelScenePart>(nRepeats + 1);
    detailedPart->centers.zeros(nRepeats + 1, 3);
    detailedPart->halfSizes.zeros(nRepeats + 1, 3);
    detailedPart->normals.zeros(nRepeats + 1, 3);
    detailedPart->intData.zeros(nRepeats + 1, 3);
    detailedPart->doubleData.zeros(nRepeats + 1, 5);
    detailedPart->identifiers.zeros(5);
    detailedPart->materialIndex.zeros(nRepeats + 1);
    detailedPart->materialTable.push_back(std::make_shared<Material>());
    detailedPart->materialTable[0]->ka[0] = 1.0;
    detailedPart->materialTable[0]->ka[1] = 2.0;
    detailedPart->materialTable[0]->ka[2] = 3.0;
    detailedPart->materialTable[0]->ka[3] = 4.0;
    detailedPart->onRayIntersectionMode = "TRANSMITTIVE";
    for (std::size_t i = 0; i < detailedPart->identifiers.n_elem; ++i) {
      detailedPart->identifiers(i) = i;
    }
    for (std::size_t i = 0; i < nRepeats + 1; ++i) {
      detailedPart->centers(i, 0) = 1.0 + static_cast<double>(i);
      detailedPart->centers(i, 1) = 2.0;
      detailedPart->centers(i, 2) = 1.0;
      detailedPart->halfSizes(i, 0) = 1.0;
      detailedPart->halfSizes(i, 1) = 1.0;
      detailedPart->halfSizes(i, 2) = 1.0;
      detailedPart->intData(i, 0) = 0;
      detailedPart->intData(i, 1) = 1;
      detailedPart->intData(i, 2) = 2;
      detailedPart->doubleData(i, 0) = 1.0;
      detailedPart->doubleData(i, 1) = 1.5;
      detailedPart->doubleData(i, 2) = 2.0;
      detailedPart->doubleData(i, 3) = 2.5;
      detailedPart->doubleData(i, 4) = 3.0;
      detailedPart->materialIndex(i) = 0;
    }

    auto trianglePart = std::make_shared<TriangleScenePart>(1);
    trianglePart->vertices.zeros(1, 9);
    trianglePart->normals.zeros(1, 9);
    trianglePart->materialIndex.zeros(1);
    trianglePart->vertices(0, 0) = 0.0;
    trianglePart->vertices(0, 1) = 0.0;
    trianglePart->vertices(0, 2) = 0.0;
    trianglePart->vertices(0, 3) = 0.0;
    trianglePart->vertices(0, 4) = 0.0;
    trianglePart->vertices(0, 5) = 4.0;
    trianglePart->vertices(0, 6) = 2.0;
    trianglePart->vertices(0, 7) = 0.0;
    trianglePart->vertices(0, 8) = 0.0;
    trianglePart->materialTable.push_back(std::make_shared<Material>());
    trianglePart->materialIndex(0) = 0;

    auto voxelPart = std::make_shared<VoxelScenePart>(1);
    voxelPart->centers.zeros(1, 3);
    voxelPart->halfSizes.zeros(1, 3);
    voxelPart->normals.zeros(1, 3);
    voxelPart->materialIndex.zeros(1);
    voxelPart->centers(0, 0) = 1.0;
    voxelPart->centers(0, 1) = 1.0;
    voxelPart->centers(0, 2) = 1.0;
    voxelPart->halfSizes(0, 0) = 1.0;
    voxelPart->halfSizes(0, 1) = 1.0;
    voxelPart->halfSizes(0, 2) = 1.0;
    voxelPart->materialTable.push_back(std::make_shared<Material>());
    voxelPart->materialIndex(0) = 0;

    scene1.parts.push_back(detailedPart);
    scene1.parts.push_back(trianglePart);
    scene1.parts.push_back(voxelPart);
    scene1.finalizeLoading(true);
    std::shared_ptr<KDGroveFactory> kdgf = scene1.getKDGroveFactory();
    scene1.writeObject(path);
    scene1.setKDGroveFactory(kdgf);
    Scene* scene2 = Scene::readObject(path);
    REQUIRE(scene2 != nullptr);
    REQUIRE(scene1.parts.size() == scene2->parts.size());
    REQUIRE(scene1.geometryRefs.size() == scene2->geometryRefs.size());
    validate(*scene1.getAABB(), *scene2->getAABB());

    for (std::size_t i = 0; i < scene1.geometryRefs.size(); ++i) {
      GeometryRef const& ref1 = scene1.geometryRefs[i];
      GeometryRef const& ref2 = scene2->geometryRefs[i];
      REQUIRE(ref1.geometryType() == ref2.geometryType());
      glm::dvec3 const c1 = ref1.centroid();
      glm::dvec3 const c2 = ref2.centroid();
      REQUIRE(c1.x == c2.x);
      REQUIRE(c1.y == c2.y);
      REQUIRE(c1.z == c2.z);
      std::shared_ptr<Material> m1 = ref1.material();
      std::shared_ptr<Material> m2 = ref2.material();
      if (m1 == nullptr || m2 == nullptr) {
        REQUIRE(m1 == nullptr);
        REQUIRE(m2 == nullptr);
      } else {
        for (std::size_t j = 0; j < 4; ++j) {
          REQUIRE(m1->ka[j] == m2->ka[j]);
        }
      }
    }

    delete scene2;
  }

  std::remove(path.c_str());
}
