#pragma once

#include "glm/vec3.hpp"
#include "glm/ext/quaternion_float.hpp"

namespace PixiePhysics
{
    struct Rigidbody
    {
        glm::vec3 linearVelocity;
    	glm::vec3 angularVelocity;

    	// Physics properties
    	glm::mat3 inverseInertiaTensor;
        float invertMass;
		float elasticity;
    	float friction;
    	glm::vec3 centerOfMass;
    };
}
