#include "UpdateTransform.hpp"

// Systems
#include "ResolveCollisions.hpp"

// Components
#include <iostream>

#include "../components/RigidBody.hpp"
#include "../components/TransformDynamic.hpp"

const glm::vec3 GRAVITY = {0.0f, -9.81f, 0.0f};

namespace PixiePhysics
{
	glm::vec3 GetGravityImpulse(const Rigidbody& body, float dt);
}

glm::vec3 PixiePhysics::GetGravityImpulse(const Rigidbody& body, const float dt)
{
	float mass = 1.0f / body.invertMass;
	return GRAVITY* mass * dt;
}


void PixiePhysics::UpdatePosition(const float dt, entt::registry &registry)
{
	auto group = registry.group<TransformDynamic, Rigidbody>();
	for (const entt::entity& entity: group)
	{
		glm::vec3 impulse = GetGravityImpulse(group.get<Rigidbody>(entity), dt);
		ApplyImpulse(group.get<Rigidbody>(entity), impulse);
		
		TransformDynamic &transform = group.get<TransformDynamic>(entity);
		Rigidbody &body = group.get<Rigidbody>(entity);
		transform.lastPosition = transform.position;

		// Velocity needs to be updated before position(because of colision restitution)
		// Can't compute acceleration because into euler formula because it transforms the movemmeent into a curve and highly complicates the sweep test
		body.linearVelocity += GRAVITY * dt;
		transform.position = transform.position + body.linearVelocity * dt;
	}
}
