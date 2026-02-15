#include <catch2/catch_test_macros.hpp>
#undef WARN
#undef INFO
#include "logging.hpp"

#include <FullWaveformPulseDetector.h>
#include <HelicopterPlatform.h>
#include <OscillatingMirrorBeamDeflector.h>
#include <Survey.h>
#include <scanner/SingleScanner.h>
#include <scene/sceneparts/DetailedVoxelScenePart.h>
#include <scene/sceneparts/TriangleScenePart.h>

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

  std::shared_ptr<TriangleScenePart> triPart =
    std::make_shared<TriangleScenePart>(1);
  triPart->vertices.row(0).zeros();
  triPart->normals.row(0).zeros();
  triPart->onRayIntersectionMode = "TRANSMITTIVE";
  baseScene->parts.push_back(triPart);

  std::shared_ptr<DetailedVoxelScenePart> dvPart =
    std::make_shared<DetailedVoxelScenePart>(1);
  dvPart->centers(0, 0) = 0.0;
  dvPart->centers(0, 1) = 0.0;
  dvPart->centers(0, 2) = 0.5;
  dvPart->halfSizes(0, 0) = 1.075;
  dvPart->halfSizes(0, 1) = 1.075;
  dvPart->halfSizes(0, 2) = 1.075;
  dvPart->intData.set_size(1, 2);
  dvPart->intData(0, 0) = 1;
  dvPart->intData(0, 1) = 2;
  dvPart->doubleData.set_size(1, 3);
  dvPart->doubleData(0, 0) = 0.1;
  dvPart->doubleData(0, 1) = 0.2;
  dvPart->doubleData(0, 2) = 0.3;
  std::shared_ptr<Material> dvMaterial = std::make_shared<Material>();
  dvMaterial->ka[0] = 1.1;
  dvMaterial->ks[1] = 1.2;
  dvPart->materialIndex.zeros(1);
  dvPart->setGeometryMaterial(0, dvMaterial);
  baseScene->parts.push_back(dvPart);

  baseScene->registerParts();
  baseScene->rebuildGeometryRefs();

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
  std::shared_ptr<TriangleScenePart> copyTriPart =
    std::dynamic_pointer_cast<TriangleScenePart>(copyScene->parts[0]);
  std::shared_ptr<DetailedVoxelScenePart> copyDvPart =
    std::dynamic_pointer_cast<DetailedVoxelScenePart>(copyScene->parts[1]);
  copyTriPart->vertices(0, 0) += 0.1;
  copyTriPart->onRayIntersectionArgument += 0.034;
  copyDvPart->geometryMaterial(0)->ks[1] += 0.1;
  copyDvPart->doubleData(0, 1) += 0.1;

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

  std::shared_ptr<TriangleScenePart> baseTriPart =
    std::dynamic_pointer_cast<TriangleScenePart>(baseScene->parts[0]);
  std::shared_ptr<DetailedVoxelScenePart> baseDvPart =
    std::dynamic_pointer_cast<DetailedVoxelScenePart>(baseScene->parts[1]);

  REQUIRE(copyTriPart->vertices(0, 0) != baseTriPart->vertices(0, 0));
  REQUIRE(copyTriPart->vertices(0, 1) == baseTriPart->vertices(0, 1));
  REQUIRE(copyTriPart->onRayIntersectionArgument !=
          baseTriPart->onRayIntersectionArgument);
  REQUIRE(copyDvPart->geometryMaterial(0)->ks[0] ==
          baseDvPart->geometryMaterial(0)->ks[0]);
  REQUIRE(copyDvPart->geometryMaterial(0)->ks[1] !=
          baseDvPart->geometryMaterial(0)->ks[1]);
  REQUIRE(copyDvPart->doubleData(0, 1) != baseDvPart->doubleData(0, 1));
  REQUIRE(copyDvPart->doubleData(0, 0) == baseDvPart->doubleData(0, 0));
}
