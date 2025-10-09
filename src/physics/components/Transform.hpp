#pragma once

#include "glm/vec3.hpp"
#include "glm/ext/quaternion_float.hpp"
#include "TransformDynamic.hpp"

namespace PixiePhysics
{
    struct Transform
    {
        glm::vec3 position;
        glm::quat rotation;

        explicit Transform(const TransformDynamic& transform)
            : position(transform.position)
            , rotation(transform.rotation)
        {
        }

        Transform(glm::vec3 pos, glm::quat rot)
            : position(pos)
            , rotation(rot)
        {}

        Transform() = default;
    };
}
