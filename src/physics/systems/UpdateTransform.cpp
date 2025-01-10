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
		transform.position = transform.position + body.linearVelocity * dt + (GRAVITY * dt * dt / 2.0f);
		body.linearVelocity += GRAVITY * dt;

		// Test when ball is changing direction
		if (std::abs(body.linearVelocity.y) < std::abs(GRAVITY.y) * dt)
		{
			float timeToZeroVelocity = (-body.linearVelocity.y / GRAVITY.y);
			glm::vec3 zeroVelocityPosition = transform.position + body.linearVelocity * timeToZeroVelocity + (GRAVITY * timeToZeroVelocity * timeToZeroVelocity / 2.0f);
			std::cout << "Position when velocity is zero: " << zeroVelocityPosition.y << std::endl;
		}
	}
}
