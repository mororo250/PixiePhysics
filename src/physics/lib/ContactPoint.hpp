//
// Created by João Mororo on 10/12/2024.
//

#pragma once

#include "entt/entity/entity.hpp"

namespace PixiePhysics
{
	struct HasCollided
	{
		bool hasCollided;
		float time; // Time of impact {0 - 1}
	};

	struct Collision
	{
		entt::entity entityA;
		entt::entity entityB;
		bool isStatic; // If the entityB is static
		float time; // Time of impact {0 - 1}
	};
}
