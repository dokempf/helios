#include <catch2/catch_test_macros.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <FullWaveformPulseDetector.h>
#include <HelicopterPlatform.h>
#include <OscillatingMirrorBeamDeflector.h>
#include <ScenePart.h>
#include <Survey.h>
#include <scanner/SingleScanner.h>

TEST_CASE("Survey Copy Test")
{
  // Build base Survey
  std::shared_ptr<Survey> survey = std::make_shared<Survey>();
  survey->name = "MySurvey";
  survey->numRuns = 1;
  survey->simSpeedFactor = 1;

  std::list<int> pulseFreqs = { 100, 30, 70 };
  survey->scanner =
    std::make_shared<SingleScanner>(0.1,
                                    glm::dvec3(2.0, 3.0, 0.0),
                                    Rotation(0.0, 0.0, 0.0, 0.0, true),
                                    pulseFreqs,
                                    4.0,
                                    "MyScanner",
                                    80.5,
                                    3.0,
                                    0.9,
                                    0.7,
                                    0.8,
                                    100,
                                    false,
                                    false,
                                    false,
                                    true,
                                    false);
  survey->scanner->setScannerHead(
    std::make_shared<ScannerHead>(glm::dvec3(0.4, 0.7, 0.1), 0.067));
  survey->scanner->setBeamDeflector(
    std::make_shared<OscillatingMirrorBeamDeflector>(
      3.141592, 1400.5, 70.8, 1));
  survey->scanner->platform = std::make_shared<HelicopterPlatform>();
  survey->scanner->setDetector(
    std::make_shared<FullWaveformPulseDetector>(survey->scanner, 1.5, 0.1));
  survey->legs.push_back(std::make_shared<Leg>());
  survey->legs[0]->mPlatformSettings = std::make_shared<PlatformSettings>();
  survey->legs[0]->mPlatformSettings->onGround = false;

  std::shared_ptr<Scene> baseScene = std::make_shared<Scene>();
  survey->scanner->platform->scene = baseScene;

  auto triPart = std::make_shared<ScenePart>();
  triPart->primitiveType = ScenePart::PrimitiveType::TRIANGLE;
  triPart->onRayIntersectionMode = "TRANSMITTIVE";
  Vertex t0(0.0, 0.0, 0.0);
  Vertex t1(0.0, 0.0, 0.0);
  Vertex t2(0.0, 0.0, 0.0);
  appendTriangleBulk(
    triPart->triangles, t0, t1, t2, std::make_shared<Material>());
  baseScene->parts.push_back(triPart);

  auto voxelPart = std::make_shared<ScenePart>();
  voxelPart->primitiveType = ScenePart::PrimitiveType::VOXEL;
  auto dvMat = std::make_shared<Material>();
  dvMat->ka[0] = 1.1;
  dvMat->ks[0] = 0.4;
  dvMat->ks[1] = 1.2;
  appendVoxelBulk(voxelPart->voxels,
                  Vertex(0.0, 0.0, 0.5),
                  2.15,
                  0,
                  0.0,
                  0.0,
                  0.0,
                  Color4f(),
                  dvMat);
  voxelPart->detailed_voxels.present.push_back(1);
  voxelPart->detailed_voxels.int_values.push_back({ 1, 2 });
  voxelPart->detailed_voxels.double_values.push_back({ 0.1, 0.2, 0.3 });
  voxelPart->detailed_voxels.max_pad.push_back(0.0);
  baseScene->parts.push_back(voxelPart);
  baseScene->registerParts();

  // Copy base Survey
  std::shared_ptr<Survey> copy = std::make_shared<Survey>(*survey, true);

  // Modify the copy
  copy->name = "CopiedSurvey";
  copy->numRuns = 0;
  copy->scanner->name = "CopiedScanner";
  Rotation& copyMRA =
    copy->scanner->getScannerHead()->getMountRelativeAttitudeByReference();
  copyMRA.setQ3(copyMRA.getQ3() + 0.1);
  copy->scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz += 1.0;
  copy->scanner->platform->cfg_device_relativeMountPosition.x += 0.01;

  HelicopterPlatform* hp = ((HelicopterPlatform*)copy->scanner->platform.get());
  glm::dvec3& speedXy = hp->getSpeedXyByReference();
  speedXy.x += 0.1;
  Rotation& r = hp->getRotationByReference();
  r.setQ2(r.getQ2() + 0.1);

  copy->scanner->getFWFSettings().minEchoWidth += 0.001;
  copy->legs[0]->mPlatformSettings->onGround = true;

  std::shared_ptr<Scene> copyScene = copy->scanner->platform->scene;
  copyScene->parts[0]->triangles.vertices[0].pos.x += 0.1;
  copyScene->parts[0]->onRayIntersectionArgument += 0.034;
  copyScene->parts[1]->voxels.materials[0]->ks[1] += 0.1;
  copyScene->parts[1]->detailed_voxels.double_values[0][1] += 0.1;

  // Validate the copy
  REQUIRE(copy->name != survey->name);
  REQUIRE(copy->numRuns != survey->numRuns);
  REQUIRE(copy->simSpeedFactor == survey->simSpeedFactor);

  REQUIRE(copy->scanner->name != survey->scanner->name);
  REQUIRE(copy->scanner->getNumTimeBins() == survey->scanner->getNumTimeBins());
  REQUIRE(copy->scanner->isCalcEchowidth() ==
          survey->scanner->isCalcEchowidth());
  REQUIRE(copy->scanner->getFWFSettings().minEchoWidth !=
          survey->scanner->getFWFSettings().minEchoWidth);
  REQUIRE(copy->scanner->getFWFSettings().apertureDiameter ==
          survey->scanner->getFWFSettings().apertureDiameter);
  REQUIRE(copy->scanner->getScannerHead()->getRotatePerSecMax() ==
          survey->scanner->getScannerHead()->getRotatePerSecMax());
  Rotation& baseMRA =
    survey->scanner->getScannerHead()->getMountRelativeAttitudeByReference();
  REQUIRE(copyMRA.getQ0() == baseMRA.getQ0());
  REQUIRE(copyMRA.getQ3() != baseMRA.getQ3());

  REQUIRE(copy->scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz ==
          survey->scanner->getBeamDeflector()->cfg_device_scanFreqMin_Hz);
  REQUIRE(copy->scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz !=
          survey->scanner->getBeamDeflector()->cfg_device_scanFreqMax_Hz);
  REQUIRE(copy->scanner->platform->cfg_device_relativeMountPosition.x !=
          survey->scanner->platform->cfg_device_relativeMountPosition.x);
  REQUIRE(copy->scanner->platform->cfg_device_relativeMountPosition.y ==
          survey->scanner->platform->cfg_device_relativeMountPosition.y);

  HelicopterPlatform* copyHp =
    static_cast<HelicopterPlatform*>(copy->scanner->platform.get());
  HelicopterPlatform* baseHp =
    static_cast<HelicopterPlatform*>(survey->scanner->platform.get());

  glm::dvec3& copySpeedXy = copyHp->getSpeedXyByReference();
  glm::dvec3& baseSpeedXy = baseHp->getSpeedXyByReference();
  REQUIRE(copySpeedXy.x != baseSpeedXy.x);
  REQUIRE(copySpeedXy.y == baseSpeedXy.y);

  Rotation& copyRot = copyHp->getRotationByReference();
  Rotation& baseRot = baseHp->getRotationByReference();
  REQUIRE(copyRot.getQ1() == baseRot.getQ1());
  REQUIRE(copyRot.getQ2() != baseRot.getQ2());

  REQUIRE(copy->legs[0]->mPlatformSettings->onGround !=
          survey->legs[0]->mPlatformSettings->onGround);
  REQUIRE(copy->legs[0]->mPlatformSettings->stopAndTurn ==
          survey->legs[0]->mPlatformSettings->stopAndTurn);

  REQUIRE(copyScene->parts[0]->triangles.vertices[0].pos.x !=
          baseScene->parts[0]->triangles.vertices[0].pos.x);
  REQUIRE(copyScene->parts[0]->triangles.vertices[0].pos.y ==
          baseScene->parts[0]->triangles.vertices[0].pos.y);
  REQUIRE(copyScene->parts[0]->onRayIntersectionArgument !=
          baseScene->parts[0]->onRayIntersectionArgument);
  REQUIRE(copyScene->parts[1]->voxels.materials[0]->ks[0] ==
          baseScene->parts[1]->voxels.materials[0]->ks[0]);
  REQUIRE(copyScene->parts[1]->voxels.materials[0]->ks[1] !=
          baseScene->parts[1]->voxels.materials[0]->ks[1]);

  REQUIRE(copyScene->parts[1]->detailed_voxels.double_values[0][1] !=
          baseScene->parts[1]->detailed_voxels.double_values[0][1]);
  REQUIRE(copyScene->parts[1]->detailed_voxels.double_values[0][0] ==
          baseScene->parts[1]->detailed_voxels.double_values[0][0]);
}
