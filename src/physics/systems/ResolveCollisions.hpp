#pragma once

#include <glm/vec3.hpp>
#include "entt/entity/registry.hpp"

namespace PixiePhysics
{
	struct ContactPoint;
	struct Rigidbody;
	
	void ResolveCollisions(float dt, entt::registry& registry);
	void ApplyImpulse(Rigidbody &body, const glm::vec3& impulse);
};
