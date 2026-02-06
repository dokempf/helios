//
// Created by miguelyermo on 5/7/21.
//

#pragma once

#include <assetloading/ScenePartGeometry.h>

/**
 * @brief Class representing a .obj loaded file
 */
class WavefrontObj
{
public:
  /**
   * @brief Bulk triangle data for the loaded OBJ.
   */
  TriangleBulk triangles{};

  WavefrontObj() = default;
  ~WavefrontObj() = default;
};
