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
  dv1.part = std::make_shared<ScenePart>();
  dv1.part->mPrimitives.push_back(&dv1);
  dv1.part->onRayIntersectionMode = "TRANSMITTIVE";
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
    Triangle t1(t1v1, t1v2, t1v3);
    t1.material = std::make_shared<Material>();
    Voxel v1(1.0, 1.0, 1.0, 1.0);
    AABB box1(glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(5.0, 5.0, 5.0));

    std::shared_ptr<ScenePart> triPart = std::make_shared<ScenePart>();
    std::vector<Primitive*> triPrims;
    triPrims.push_back(t1.clone());
    triPart->setPrimitives(triPrims);

    std::shared_ptr<ScenePart> voxelPart = std::make_shared<ScenePart>();
    std::vector<Primitive*> voxelPrims;
    voxelPrims.push_back(dv1.clone());
    voxelPrims.push_back(v1.clone());
    for (std::size_t i = 0; i < nRepeats; i++) {
      voxelPrims.push_back(dv1.clone());
    }
    voxelPart->setPrimitives(voxelPrims);

    scene1.parts.push_back(triPart);
    scene1.parts.push_back(voxelPart);
    scene1.primitives.insert(scene1.primitives.end(),
                             triPart->mPrimitives.begin(),
                             triPart->mPrimitives.end());
    scene1.primitives.insert(scene1.primitives.end(),
                             voxelPart->mPrimitives.begin(),
                             voxelPart->mPrimitives.end());
    scene1.primitives.push_back(box1.clone());
    scene1.finalizeLoading(true);
    std::shared_ptr<KDGroveFactory> kdgf = scene1.getKDGroveFactory();
    scene1.writeObject(path);
    scene1.setKDGroveFactory(kdgf);
    Scene* scene2 = Scene::readObject(path);

    size_t const triIndex = 0;
    size_t const voxelStart = triPart->mPrimitives.size();
    size_t const dvIndex = voxelStart;
    size_t const vIndex = voxelStart + 1;
    size_t const repeatStart = voxelStart + 2;
    size_t const aabbIndex = voxelStart + voxelPart->mPrimitives.size();

    std::unique_ptr<DetailedVoxel> dv1Copy(
      dynamic_cast<DetailedVoxel*>(scene1.primitives[dvIndex]->clone()));
    std::unique_ptr<DetailedVoxel> dv2Copy(
      dynamic_cast<DetailedVoxel*>(scene2->primitives[dvIndex]->clone()));
    REQUIRE(dv1Copy != nullptr);
    REQUIRE(dv2Copy != nullptr);
    validate(*dv1Copy, *dv2Copy);

    std::unique_ptr<Triangle> t1Copy(
      dynamic_cast<Triangle*>(scene1.primitives[triIndex]->clone()));
    std::unique_ptr<Triangle> t2Copy(
      dynamic_cast<Triangle*>(scene2->primitives[triIndex]->clone()));
    REQUIRE(t1Copy != nullptr);
    REQUIRE(t2Copy != nullptr);
    validate(*t1Copy, *t2Copy);

    std::unique_ptr<Voxel> v1Copy(
      dynamic_cast<Voxel*>(scene1.primitives[vIndex]->clone()));
    std::unique_ptr<Voxel> v2Copy(
      dynamic_cast<Voxel*>(scene2->primitives[vIndex]->clone()));
    REQUIRE(v1Copy != nullptr);
    REQUIRE(v2Copy != nullptr);
    validate(*v1Copy, *v2Copy);

    AABB* aabb1 = dynamic_cast<AABB*>(scene1.primitives[aabbIndex]);
    AABB* aabb2 = dynamic_cast<AABB*>(scene2->primitives[aabbIndex]);
    REQUIRE(aabb1 != nullptr);
    REQUIRE(aabb2 != nullptr);
    validate(*aabb1, *aabb2);

    for (size_t i = 0; i < nRepeats; i++) {
      std::unique_ptr<DetailedVoxel> dv1Repeat(dynamic_cast<DetailedVoxel*>(
        scene1.primitives[repeatStart + i]->clone()));
      std::unique_ptr<DetailedVoxel> dv2Repeat(dynamic_cast<DetailedVoxel*>(
        scene2->primitives[repeatStart + i]->clone()));
      REQUIRE(dv1Repeat != nullptr);
      REQUIRE(dv2Repeat != nullptr);
      validate(*dv1Repeat, *dv2Repeat);
    }

    delete scene2;
  }

  // Cleanup
  std::remove(path.c_str());
}
