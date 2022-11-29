/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#if defined(__APPLE__)
  #include <OpenGL/gl.h>
  #include <GLUT/glut.h>
#elif not defined(_WIN32)
  #include <GL/glew.h>
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif

#include <iostream>
#include <vector>
#include <tuple>

#include <ignition/common/Console.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/rendering.hh>

#include "example_config.hh"
#include "GlutWindow.hh"

using namespace ignition;
using namespace rendering;

const std::string RESOURCE_PATH =
    common::joinPaths(std::string(PROJECT_BINARY_PATH), "media");

//////////////////////////////////////////////////
void buildScene(ScenePtr _scene)
{
  // initialize _scene
  _scene->SetAmbientLight(0.3, 0.3, 0.3);
  _scene->SetBackgroundColor(0.3, 0.3, 0.3);
  VisualPtr root = _scene->RootVisual();

  // create directional light
  DirectionalLightPtr light0 = _scene->CreateDirectionalLight();
  light0->SetDirection(0.5, 0.5, -1);
  light0->SetDiffuseColor(0.8, 0.8, 0.8);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light0);

  //! [create a mesh]
  VisualPtr mesh = _scene->CreateVisual();
  mesh->SetLocalPosition(3, 0, 0);
  mesh->SetLocalRotation(1.5708, 0, 2.0);
  MeshDescriptor descriptor;
  descriptor.meshName = common::joinPaths(RESOURCE_PATH, "duck.dae");
  common::MeshManager *meshManager = common::MeshManager::Instance();
  descriptor.mesh = meshManager->Load(descriptor.meshName);
  MeshPtr meshGeom = _scene->CreateMesh(descriptor);
  mesh->AddGeometry(meshGeom);
  root->AddChild(mesh);
  //! [create a mesh]

  // create gray material
  MaterialPtr gray = _scene->CreateMaterial();
  gray->SetAmbient(1, 1, 1);
  gray->SetDiffuse(1, 1, 1);
  gray->SetSpecular(1, 1, 1);

  //! [create grid visual]
  VisualPtr grid = _scene->CreateVisual();
  GridPtr gridGeom = _scene->CreateGrid();
  gridGeom->SetCellCount(20);
  gridGeom->SetCellLength(1);
  gridGeom->SetVerticalCellCount(0);
  grid->AddGeometry(gridGeom);
  grid->SetLocalPosition(3, 0, 0.0);
  grid->SetMaterial(gray);
  root->AddChild(grid);
  //! [create grid visual]

  //! [create camera]
  CameraPtr camera = _scene->CreateCamera("camera");
  camera->SetLocalPosition(-5.0, -5.0, 3);
  camera->SetLocalRotation(0.0, 0.0, 1.0);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(IGN_PI / 2);
  root->AddChild(camera);
  //! [create camera]

  //! [create depth camera]
  DepthCameraPtr depthCamera =_scene->CreateDepthCamera("depth_camera");
  depthCamera->SetLocalPosition(-5.0, -5.0, 3);
  depthCamera->SetLocalRotation(0.0, 0.0, 1.0);
  depthCamera->SetImageWidth(800);
  depthCamera->SetImageHeight(600);
  depthCamera->SetAspectRatio(1.333);
  depthCamera->SetHFOV(IGN_PI / 2);
  depthCamera->SetImageFormat(PixelFormat::PF_FLOAT32_RGBA);
  depthCamera->SetNearClipPlane(0.15);
  depthCamera->SetFarClipPlane(10.0);
  depthCamera->SetAntiAliasing(2);
  depthCamera->CreateDepthTexture();
  root->AddChild(depthCamera);
  //! [create depth camera]

  // create particle material
  MaterialPtr rainMaterial = _scene->CreateMaterial();
  rainMaterial->SetDiffuse(0.7, 0.7, 0.7);
  rainMaterial->SetTexture(RESOURCE_PATH + "/rain_transparent.png");
  rainMaterial->SetAlphaFromTexture(true);

  MaterialPtr smokeMaterial = _scene->CreateMaterial();
  smokeMaterial->SetDiffuse(0.7, 0.7, 0.7);
  smokeMaterial->SetTexture(RESOURCE_PATH + "/smoke.png");
  smokeMaterial->SetAlphaFromTexture(true);

  // Rain from a grid of points
  std::vector<ParticleEmitterPtr> gridParticleVec;
  int width = 3;
  int length = 3;
  double interval = 3.0;
  double height = 15.0;

  for (int i = 0; i < width; i++) {
    for (int j = 0; j < length; j++) {
        // Rain from a point
        ParticleEmitterPtr emitter = _scene->CreateParticleEmitter();
        emitter->SetType(EM_POINT);
        emitter->SetLocalPose({i*interval, j*interval, height, 0.0, 3.14/2, 0.0});
        emitter->SetRate(10);
        emitter->SetParticleSize({1, 1, 1});
        emitter->SetLifetime(2);
        emitter->SetVelocityRange(10, 20);
        emitter->SetMaterial(rainMaterial);
        emitter->SetColorRangeImage(RESOURCE_PATH + "/smokecolors.png");
        emitter->SetParticleScatterRatio(0.0);

        emitter->SetScaleRate(10);
        emitter->SetEmitting(true);
        root->AddChild(emitter);
        gridParticleVec.push_back(emitter);
    }
  }
  //! [create particle emitter]
  // Rain from a point
  /*
  ParticleEmitterPtr emitter = _scene->CreateParticleEmitter();
  emitter->SetType(EM_POINT);
  emitter->SetLocalPose({0, 0, 7.0, 0.0, 3.14/2, 0.0});
  emitter->SetRate(10);
  emitter->SetParticleSize({1, 1, 1});
  emitter->SetLifetime(2);
  emitter->SetVelocityRange(10, 20);
  emitter->SetMaterial(rainMaterial);
  emitter->SetColorRangeImage(RESOURCE_PATH + "/smokecolors.png");
  emitter->SetScaleRate(10);
  emitter->SetEmitting(true);
  root->AddChild(emitter);
  */

  // area emitter
  // rain from an area
//   ParticleEmitterPtr areaEmitter = _scene->CreateParticleEmitter();
//   areaEmitter->SetType(EM_BOX);
//   areaEmitter->SetEmitterSize({3.0, 3.0, 3.0});
//   areaEmitter->SetLocalPose({5, 5, 7, 0, 3.14/2, 0});
//   areaEmitter->SetRate(10);
//   areaEmitter->SetParticleSize({2, 2, 2});
//   areaEmitter->SetLifetime(2);
//   areaEmitter->SetVelocityRange(10, 20);
//   areaEmitter->SetMaterial(rainMaterial);
//   areaEmitter->SetColorRangeImage(RESOURCE_PATH + "/smokecolors.png");
//   areaEmitter->SetScaleRate(5);
//   areaEmitter->SetEmitting(true);
//   root->AddChild(areaEmitter);
}

//////////////////////////////////////////////////
std::tuple<CameraPtr, CameraPtr> createCameras(const std::string &_engineName,
    const std::map<std::string, std::string>& _params)
{
  // create and populate scene
  RenderEngine *engine = rendering::engine(_engineName, _params);
  if (!engine)
  {
    ignwarn << "Engine '" << _engineName
            << "' is not supported" << std::endl;
    return std::make_tuple(CameraPtr(), CameraPtr());
  }
  ScenePtr scene = engine->CreateScene("scene");
  buildScene(scene);

  // return camera sensor
  SensorPtr sensor = scene->SensorByName("camera");
  SensorPtr depth_sensor = scene->SensorByName("depth_camera");

  std::tuple<CameraPtr, CameraPtr> sensors
    = std::make_tuple(std::dynamic_pointer_cast<Camera>(sensor),
    std::dynamic_pointer_cast<Camera>(depth_sensor)
  );
  return sensors;
}

//////////////////////////////////////////////////
int main(int _argc, char** _argv)
{
  glutInit(&_argc, _argv);

  // Expose engine name to command line because we can't instantiate both
  // ogre and ogre2 at the same time
  std::string ogreEngineName("ogre2");
  if (_argc > 1)
  {
    ogreEngineName = _argv[1];
  }

  GraphicsAPI graphicsApi = GraphicsAPI::OPENGL;
  if (_argc > 2)
  {
    graphicsApi = GraphicsAPIUtils::Set(std::string(_argv[2]));
  }

  common::Console::SetVerbosity(4);
  std::vector<std::string> engineNames;
  std::vector<CameraPtr> cameras;

  engineNames.push_back(ogreEngineName);

  for (auto engineName : engineNames)
  {
    try
    {
      std::map<std::string, std::string> params;
      if (engineName.compare("ogre2") == 0
          && graphicsApi == GraphicsAPI::METAL)
      {
        params["metal"] = "1";
      }

      auto _cameras = createCameras(engineName, params);
      auto camera = std::get<0>(_cameras);
      auto depth_camera = std::get<1>(_cameras);
      if (camera)
      {
        cameras.push_back(camera);
        std::cout << "Added RGB Camera" << std::endl;
      }

      if (depth_camera)
      {
        cameras.push_back(depth_camera);
        std::cout << "Added Depth Camera" << std::endl;
      }
    }
    catch (...)
    {
      // std::cout << ex.what() << std::endl;
      std::cerr << "Error starting up: " << engineName << std::endl;
    }
  }
  run(cameras);
  return 0;
}
