#pragma once

#include <glm/vec3.hpp>
#include "entt/entity/registry.hpp"

namespace PixiePhysics
{
	struct StaticBody;
	struct ContactPoint;
	struct Rigidbody;
	
	void ResolveCollisions(float dt, entt::registry& registry);
	void ApplyFriction(Rigidbody &body, const StaticBody& staticBody, const glm::vec3& normal, const glm::vec3& localColPoint, const glm::vec3& vel);
	void ApplyImpulseLinear(Rigidbody &body, const glm::vec3& impulse);
	void ApplyImpulseAngular(Rigidbody &body, const glm::vec3& impulse, const glm::vec3& impulsePoint);
	void ApplyImpulse(Rigidbody &body, const glm::vec3& impulse, const glm::vec3& impulsePoint);
};
