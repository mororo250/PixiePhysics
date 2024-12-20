//
// Created by João Mororo on 10/12/2024.
//

#pragma once

#include "entt/entity/entity.hpp"
#include "glm/vec3.hpp"

namespace PixiePhysics
{
	struct ContactPoint
	{
		entt::entity entityA;
		entt::entity entityB;
		
		glm::vec3 point;
		glm::vec3 normal;
		float timeOfImpact;
		float separationDistance;
	};
}
