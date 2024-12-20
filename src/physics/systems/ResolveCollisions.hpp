#pragma once

#include "../components/SphereShape.hpp"
#include "../components/RigidBody.hpp"
#include "../components/Transform.hpp"
#include "entt/entity/registry.hpp"

namespace PixiePhysics
{
	struct ContactPoint;
	
	void ResolveCollisions(float deltaTime, entt::registry& registry);
	void ApplyImpulse(Rigidbody &body, const glm::vec3& impulse);
};
