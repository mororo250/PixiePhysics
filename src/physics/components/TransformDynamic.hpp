#pragma once

#include "glm/vec3.hpp"
#include "glm/ext/quaternion_float.hpp"

namespace PixiePhysics
{
    struct TransformDynamic
    {
        glm::vec3 position;
        glm::quat rotation;
	    
	    glm::vec3 lastPosition;
	    glm::quat lastRotation;
    };
}
