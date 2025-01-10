#pragma once

#include "glm/vec3.hpp"
#include "glm/ext/quaternion_float.hpp"

namespace PixiePhysics
{
    struct TransformStatic
    {
        glm::vec3 position;
        glm::quat rotation;
    };
}
