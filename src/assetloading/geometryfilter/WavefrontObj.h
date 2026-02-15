//
// Created by miguelyermo on 5/7/21.
//

#pragma once

#include <Vertex.h>

#include <memory>
#include <vector>

class Material;

/**
 * @brief Class representing a .obj loaded file
 */
class WavefrontObj
{
public:
  /**
   * @brief Triangle record extracted from an OBJ face.
   */
  struct TriangleRecord
  {
    Vertex verts[3];
    std::shared_ptr<Material> material = nullptr;
  };

  /**
   * @brief Triangles loaded from the OBJ file.
   */
  std::vector<TriangleRecord> triangles{};

  WavefrontObj() = default;
  ~WavefrontObj();
};
