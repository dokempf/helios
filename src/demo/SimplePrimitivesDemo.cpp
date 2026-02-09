#ifdef PCL_BINDING

#include <DynMotion.h>
#include <MathConstants.h>
#include <assetloading/ScenePartGeometry.h>
#include <demo/SimplePrimitivesDemo.h>
#include <rigidmotion/RigidMotionR3Factory.h>
#include <scene/dynamic/DynMovingObject.h>
#include <scene/dynamic/DynObject.h>
#include <visualhelios/VHSimpleCanvas.h>

using namespace std::chrono_literals;
using HeliosDemos::SimplePrimitivesDemo;
using rigidmotion::RigidMotionR3Factory;
using std::make_shared;
using std::shared_ptr;
using std::vector;
using visualhelios::VHDynObjectXYZRGBAdapter;
using visualhelios::VHSimpleCanvas;

namespace {
void
setTriangleNormals(Vertex& v0, Vertex& v1, Vertex& v2)
{
  glm::dvec3 e1 = v1.pos - v0.pos;
  glm::dvec3 e2 = v2.pos - v0.pos;
  glm::dvec3 normal = glm::cross(e1, e2);
  double const len = glm::length(normal);
  if (len > 0.0)
    normal = normal / len;
  v0.normal = normal;
  v1.normal = normal;
  v2.normal = normal;
}

void
appendTriangle(ScenePart& part,
               Vertex v0,
               Vertex v1,
               Vertex v2,
               std::shared_ptr<Material> const& mat)
{
  setTriangleNormals(v0, v1, v2);
  appendTriangleBulk(part.triangles, v0, v1, v2, mat);
}

Vertex
makeColoredVertex(double x, double y, double z, Color4f const& color)
{
  Vertex v(x, y, z);
  v.color = color;
  return v;
}
} // namespace

// ***  R U N  *** //
// *************** //
void
SimplePrimitivesDemo::run()
{
  std::cout << "RUNNING SIMPLE PRIMITIVES DEMO ..." << std::endl;

  // Build objects
  shared_ptr<DynObject> mobileStructure = buildMobileStructure();
  shared_ptr<VHDynObjectXYZRGBAdapter> adaptedMobileStructure =
    make_shared<VHDynObjectXYZRGBAdapter>(*mobileStructure);
  adaptedMobileStructure->setRenderingNormals(true);
  shared_ptr<DynObject> fixedStructure = buildFixedStructure();
  shared_ptr<VHDynObjectXYZRGBAdapter> adaptedFixedStructure =
    make_shared<VHDynObjectXYZRGBAdapter>(*fixedStructure);
  adaptedFixedStructure->setRenderingNormals(true);
  shared_ptr<DynObject> helicalStructure = buildHelicalStructure();
  shared_ptr<VHDynObjectXYZRGBAdapter> adaptedHelicalStructure =
    make_shared<VHDynObjectXYZRGBAdapter>(*helicalStructure);
  adaptedHelicalStructure->setRenderingNormals(true);
  shared_ptr<DynObject> staticStructure = buildStaticStructure();
  shared_ptr<VHDynObjectXYZRGBAdapter> adaptedStaticStructure =
    make_shared<VHDynObjectXYZRGBAdapter>(*staticStructure);
  adaptedStaticStructure->setRenderingNormals(true);
  shared_ptr<DynObject> groundStructure = buildGroundStructure();
  shared_ptr<VHDynObjectXYZRGBAdapter> adaptedGroundStructure =
    make_shared<VHDynObjectXYZRGBAdapter>(*groundStructure);
  adaptedGroundStructure->setRenderingNormals(false);

  // Build canvas
  VHSimpleCanvas canvas("Simple primitives demo");
  canvas.appendDynObj(adaptedMobileStructure);
  canvas.appendDynObj(adaptedFixedStructure);
  canvas.appendDynObj(adaptedHelicalStructure);
  canvas.appendDynObj(adaptedStaticStructure);
  canvas.appendDynObj(adaptedGroundStructure);
  canvas.setTimeBetweenUpdates(20);
  canvas.setRenderingNormals(true);

  // Define initial transformations
  RigidMotionR3Factory rm3f;
  shared_ptr<DynMovingObject> dmoMobile =
    std::static_pointer_cast<DynMovingObject>(mobileStructure);
  RigidMotion rm = rm3f.makeRotationZ(PI_HALF);
  dmoMobile->pushPositionMotion(make_shared<DynMotion>(rm));
  dmoMobile->pushNormalMotion(make_shared<DynMotion>(rm));
  dmoMobile->pushPositionMotion(
    make_shared<DynMotion>(rm3f.makeTranslation(arma::colvec("-40;0;1"))));
  shared_ptr<DynMovingObject> dmoFixed =
    std::static_pointer_cast<DynMovingObject>(fixedStructure);
  dmoFixed->pushPositionMotion(
    make_shared<DynMotion>(rm3f.makeTranslation(arma::colvec("10;10;4"))));
  shared_ptr<DynMovingObject> dmoHelical =
    std::static_pointer_cast<DynMovingObject>(helicalStructure);
  rm = rm3f.makeRotationX(PI_HALF);
  dmoHelical->pushPositionMotion(make_shared<DynMotion>(rm));
  dmoHelical->pushNormalMotion(make_shared<DynMotion>(rm));
  dmoHelical->pushPositionMotion(
    make_shared<DynMotion>(rm3f.makeTranslation(arma::colvec("0;5;1"))));
  shared_ptr<DynMovingObject> dmoStatic =
    std::static_pointer_cast<DynMovingObject>(staticStructure);
  dmoStatic->pushPositionMotion(
    make_shared<DynMotion>(rm3f.makeTranslation(arma::colvec("0;0;10"))));

  // Define dynamic behavior function
  canvas.setDynamicUpdateFunction(
    [&rm3f](const vector<shared_ptr<VHDynObjectXYZRGBAdapter>>& objs) -> void {
      DynMovingObject& dmoMobile =
        static_cast<DynMovingObject&>(objs[0]->getDynObj());
      DynMovingObject& dmoFixed =
        static_cast<DynMovingObject&>(objs[1]->getDynObj());
      DynMovingObject& dmoHelical =
        static_cast<DynMovingObject&>(objs[2]->getDynObj());
      RigidMotion rm = rm3f.makeRotationZ(0.01);
      dmoMobile.pushPositionMotion(make_shared<DynMotion>(rm));
      dmoMobile.pushNormalMotion(make_shared<DynMotion>(rm));
      dmoFixed.pushPositionMotion(make_shared<DynMotion>(
        rm3f.makeTranslation(arma::colvec("-10;-10;-4"))));
      rm = rm3f.makeRotationZ(0.05);
      dmoFixed.pushPositionMotion(make_shared<DynMotion>(rm));
      dmoFixed.pushNormalMotion(make_shared<DynMotion>(rm));
      dmoFixed.pushPositionMotion(
        make_shared<DynMotion>(rm3f.makeTranslation(arma::colvec("10;10;4"))));
      std::vector<Vertex*> helixVerts = dmoHelical.getAllVertices();
      if (!helixVerts.empty() && helixVerts[0]->getZ() >= 20.0)
        dmoHelical.pushPositionMotion(make_shared<DynMotion>(
          rm3f.makeTranslation(arma::colvec("0;0;-20"))));
      dmoHelical.pushPositionMotion(
        make_shared<DynMotion>(rm3f.makeHelicalZ(0.15, 0.034)));
      dmoHelical.pushNormalMotion(
        make_shared<DynMotion>(rm3f.makeRotationZ(0.15)));
    });

  // Render canvas
  canvas.show();

  std::cout << "FINISHED SIMPLE PRIMITIVES DEMO!" << std::endl;
}

// ***  U T I L  *** //
// ***************** //
shared_ptr<DynObject>
SimplePrimitivesDemo::buildMobileStructure()
{
  shared_ptr<DynObject> dynObj =
    make_shared<DynMovingObject>("mobileStructure");
  dynObj->primitiveType = ScenePart::PrimitiveType::TRIANGLE;
  std::shared_ptr<Material> mat = std::make_shared<Material>();
  Color4f color(0.5f, 0.0f, 0.0f, 1.0f);

  // Bottom surface
  appendTriangle(*dynObj,
                 makeColoredVertex(-4.0, 2.0, -1.0, color),
                 makeColoredVertex(4.0, -2.0, -1.0, color),
                 makeColoredVertex(-4.0, -2.0, -1.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-4.0, 2.0, -1.0, color),
                 makeColoredVertex(4.0, 2.0, -1.0, color),
                 makeColoredVertex(4.0, -2.0, -1.0, color),
                 mat);
  // Top surface
  appendTriangle(*dynObj,
                 makeColoredVertex(-4.0, 2.0, 1.0, color),
                 makeColoredVertex(-4.0, -2.0, 1.0, color),
                 makeColoredVertex(4.0, -2.0, 1.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-4.0, 2.0, 1.0, color),
                 makeColoredVertex(4.0, -2.0, 1.0, color),
                 makeColoredVertex(4.0, 2.0, 1.0, color),
                 mat);
  // Left surface
  appendTriangle(*dynObj,
                 makeColoredVertex(-4.0, -2.0, -1.0, color),
                 makeColoredVertex(-4.0, -2.0, 1.0, color),
                 makeColoredVertex(-4.0, 2.0, 1.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-4.0, -2.0, -1.0, color),
                 makeColoredVertex(-4.0, 2.0, 1.0, color),
                 makeColoredVertex(-4.0, 2.0, -1.0, color),
                 mat);
  // Right surface
  appendTriangle(*dynObj,
                 makeColoredVertex(4.0, -2.0, -1.0, color),
                 makeColoredVertex(4.0, 2.0, 1.0, color),
                 makeColoredVertex(4.0, -2.0, 1.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(4.0, -2.0, -1.0, color),
                 makeColoredVertex(4.0, 2.0, -1.0, color),
                 makeColoredVertex(4.0, 2.0, 1.0, color),
                 mat);
  // Front face
  appendTriangle(*dynObj,
                 makeColoredVertex(-4.0, -2.0, -1.0, color),
                 makeColoredVertex(4.0, -2.0, 1.0, color),
                 makeColoredVertex(-4.0, -2.0, 1.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-4.0, -2.0, -1.0, color),
                 makeColoredVertex(4.0, -2.0, -1.0, color),
                 makeColoredVertex(4.0, -2.0, 1.0, color),
                 mat);
  // Back face
  appendTriangle(*dynObj,
                 makeColoredVertex(-4.0, 2.0, -1.0, color),
                 makeColoredVertex(-4.0, 2.0, 1.0, color),
                 makeColoredVertex(4.0, 2.0, 1.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-4.0, 2.0, -1.0, color),
                 makeColoredVertex(4.0, 2.0, 1.0, color),
                 makeColoredVertex(4.0, 2.0, -1.0, color),
                 mat);

  dynObj->computeCentroid();
  return dynObj;
}
shared_ptr<DynObject>
SimplePrimitivesDemo::buildFixedStructure()
{
  shared_ptr<DynObject> dynObj = make_shared<DynMovingObject>("fixedStructure");
  dynObj->primitiveType = ScenePart::PrimitiveType::TRIANGLE;
  std::shared_ptr<Material> mat = std::make_shared<Material>();
  Color4f color(0.0f, 0.5f, 0.0f, 1.0f);

  // Bottom surface
  appendTriangle(*dynObj,
                 makeColoredVertex(-2.0, 2.0, -4.0, color),
                 makeColoredVertex(2.0, -2.0, -4.0, color),
                 makeColoredVertex(-2.0, -2.0, -4.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-2.0, 2.0, -4.0, color),
                 makeColoredVertex(2.0, 2.0, -4.0, color),
                 makeColoredVertex(2.0, -2.0, -4.0, color),
                 mat);
  // Top surface
  appendTriangle(*dynObj,
                 makeColoredVertex(-2.0, 2.0, 4.0, color),
                 makeColoredVertex(-2.0, -2.0, 4.0, color),
                 makeColoredVertex(2.0, -2.0, 4.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-2.0, 2.0, 4.0, color),
                 makeColoredVertex(2.0, -2.0, 4.0, color),
                 makeColoredVertex(2.0, 2.0, 4.0, color),
                 mat);
  // Left surface
  appendTriangle(*dynObj,
                 makeColoredVertex(-2.0, -2.0, -4.0, color),
                 makeColoredVertex(-2.0, -2.0, 4.0, color),
                 makeColoredVertex(-2.0, 2.0, 4.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-2.0, -2.0, -4.0, color),
                 makeColoredVertex(-2.0, 2.0, 4.0, color),
                 makeColoredVertex(-2.0, 2.0, -4.0, color),
                 mat);
  // Right surface
  appendTriangle(*dynObj,
                 makeColoredVertex(2.0, -2.0, -4.0, color),
                 makeColoredVertex(2.0, 2.0, 4.0, color),
                 makeColoredVertex(2.0, -2.0, 4.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(2.0, -2.0, -4.0, color),
                 makeColoredVertex(2.0, 2.0, -4.0, color),
                 makeColoredVertex(2.0, 2.0, 4.0, color),
                 mat);
  // Back face
  appendTriangle(*dynObj,
                 makeColoredVertex(-2.0, -2.0, -4.0, color),
                 makeColoredVertex(2.0, -2.0, 4.0, color),
                 makeColoredVertex(-2.0, -2.0, 4.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-2.0, -2.0, -4.0, color),
                 makeColoredVertex(2.0, -2.0, -4.0, color),
                 makeColoredVertex(2.0, -2.0, 4.0, color),
                 mat);
  // Front face
  appendTriangle(*dynObj,
                 makeColoredVertex(-2.0, 2.0, -4.0, color),
                 makeColoredVertex(-2.0, 2.0, 4.0, color),
                 makeColoredVertex(2.0, 2.0, 4.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-2.0, 2.0, -4.0, color),
                 makeColoredVertex(2.0, 2.0, 4.0, color),
                 makeColoredVertex(2.0, 2.0, -4.0, color),
                 mat);
  dynObj->computeCentroid();
  return dynObj;
}
shared_ptr<DynObject>
SimplePrimitivesDemo::buildHelicalStructure()
{
  shared_ptr<DynObject> dynObj =
    make_shared<DynMovingObject>("helicalStructure");
  dynObj->primitiveType = ScenePart::PrimitiveType::TRIANGLE;
  std::shared_ptr<Material> mat = std::make_shared<Material>();
  Color4f color(0.6f, 0.6f, 0.0f, 1.0f);

  // Upper surface
  appendTriangle(*dynObj,
                 makeColoredVertex(-1.0, -1.0, 0.0, color),
                 makeColoredVertex(1.0, -1.0, 0.0, color),
                 makeColoredVertex(0.0, 0.0, 2.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(0.0, 0.0, 2.0, color),
                 makeColoredVertex(1.0, -1.0, 0.0, color),
                 makeColoredVertex(0.0, 1.0, 0.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-1.0, -1.0, 0.0, color),
                 makeColoredVertex(0.0, 0.0, 2.0, color),
                 makeColoredVertex(0.0, 1.0, 0.0, color),
                 mat);
  // Lower surface
  appendTriangle(*dynObj,
                 makeColoredVertex(-1.0, -1.0, 0.0, color),
                 makeColoredVertex(0.0, 0.0, -2.0, color),
                 makeColoredVertex(1.0, -1.0, 0.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(0.0, 0.0, -2.0, color),
                 makeColoredVertex(0.0, 1.0, 0.0, color),
                 makeColoredVertex(1.0, -1.0, 0.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-1.0, -1.0, 0.0, color),
                 makeColoredVertex(0.0, 1.0, 0.0, color),
                 makeColoredVertex(0.0, 0.0, -2.0, color),
                 mat);
  dynObj->computeCentroid();
  return dynObj;
}
shared_ptr<DynObject>
SimplePrimitivesDemo::buildStaticStructure()
{
  shared_ptr<DynObject> dynObj =
    make_shared<DynMovingObject>("staticStructure");
  dynObj->primitiveType = ScenePart::PrimitiveType::TRIANGLE;
  std::shared_ptr<Material> mat = std::make_shared<Material>();
  Color4f color(0.2f, 0.0f, 0.4f, 1.0f);

  // Bottom surface
  appendTriangle(*dynObj,
                 makeColoredVertex(-1.0, 1.0, -10.0, color),
                 makeColoredVertex(1.0, -1.0, -10.0, color),
                 makeColoredVertex(-1.0, -1.0, -10.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-1.0, 1.0, -10.0, color),
                 makeColoredVertex(1.0, 1.0, -10.0, color),
                 makeColoredVertex(1.0, -1.0, -10.0, color),
                 mat);
  // Top surface
  appendTriangle(*dynObj,
                 makeColoredVertex(-1.0, 1.0, 10.0, color),
                 makeColoredVertex(-1.0, -1.0, 10.0, color),
                 makeColoredVertex(1.0, -1.0, 10.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-1.0, 1.0, 10.0, color),
                 makeColoredVertex(1.0, -1.0, 10.0, color),
                 makeColoredVertex(1.0, 1.0, 10.0, color),
                 mat);
  // Left surface
  appendTriangle(*dynObj,
                 makeColoredVertex(-1.0, -1.0, -10.0, color),
                 makeColoredVertex(-1.0, -1.0, 10.0, color),
                 makeColoredVertex(-1.0, 1.0, 10.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-1.0, -1.0, -10.0, color),
                 makeColoredVertex(-1.0, 1.0, 10.0, color),
                 makeColoredVertex(-1.0, 1.0, -10.0, color),
                 mat);
  // Right surface
  appendTriangle(*dynObj,
                 makeColoredVertex(1.0, -1.0, -10.0, color),
                 makeColoredVertex(1.0, 1.0, 10.0, color),
                 makeColoredVertex(1.0, -1.0, 10.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(1.0, -1.0, -10.0, color),
                 makeColoredVertex(1.0, 1.0, -10.0, color),
                 makeColoredVertex(1.0, 1.0, 10.0, color),
                 mat);
  // Front face
  appendTriangle(*dynObj,
                 makeColoredVertex(-1.0, -1.0, -10.0, color),
                 makeColoredVertex(1.0, -1.0, 10.0, color),
                 makeColoredVertex(-1.0, -1.0, 10.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-1.0, -1.0, -10.0, color),
                 makeColoredVertex(1.0, -1.0, -10.0, color),
                 makeColoredVertex(1.0, -1.0, 10.0, color),
                 mat);
  // Back face
  appendTriangle(*dynObj,
                 makeColoredVertex(-1.0, 1.0, -10.0, color),
                 makeColoredVertex(-1.0, 1.0, 10.0, color),
                 makeColoredVertex(1.0, 1.0, 10.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(-1.0, 1.0, -10.0, color),
                 makeColoredVertex(1.0, 1.0, 10.0, color),
                 makeColoredVertex(1.0, 1.0, -10.0, color),
                 mat);
  dynObj->computeCentroid();
  return dynObj;
}

shared_ptr<DynObject>
SimplePrimitivesDemo::buildGroundStructure()
{
  shared_ptr<DynObject> dynObj =
    make_shared<DynMovingObject>("groundStructure");
  dynObj->primitiveType = ScenePart::PrimitiveType::TRIANGLE;
  std::shared_ptr<Material> mat = std::make_shared<Material>();
  Color4f color(0.5f, 0.5f, 0.5f, 1.0f);

  // Ground surface
  appendTriangle(*dynObj,
                 makeColoredVertex(-50.0, -50.0, 0.0, color),
                 makeColoredVertex(-50.0, 50.0, 0.0, color),
                 makeColoredVertex(50.0, 50.0, 0.0, color),
                 mat);
  appendTriangle(*dynObj,
                 makeColoredVertex(50.0, 50.0, 0.0, color),
                 makeColoredVertex(50.0, -50.0, 0.0, color),
                 makeColoredVertex(-50.0, -50.0, 0.0, color),
                 mat);
  dynObj->computeCentroid();
  return dynObj;
}

#endif
