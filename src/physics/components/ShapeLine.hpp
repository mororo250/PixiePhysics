#pragma once

#include "glm/vec3.hpp"
#include "glm/ext/quaternion_float.hpp"

namespace PixiePhysics
{
    struct ShapeLine
    {
        glm::vec3 startPos;
        glm::vec3 endPos;
    };
}
