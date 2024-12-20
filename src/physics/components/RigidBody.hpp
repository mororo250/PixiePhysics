#pragma once

#include "glm/vec3.hpp"
#include "glm/ext/quaternion_float.hpp"

namespace PixiePhysics
{
    struct Rigidbody
    {
        glm::vec3 linearVelocity;
        float invertMass;
		float elasticity;
    };
}
