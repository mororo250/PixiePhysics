#include "UpdateTransform.hpp"

// Systems
#include "ResolveCollisions.hpp"

// Components
#include "../components/RigidBody.hpp"
#include "../components/Transform.hpp"

const float GRAVITY = 9.81f;

namespace PixiePhysics
{
	glm::vec3 GetGravityImpulse(const Rigidbody& body, const float deltaTime);
}

glm::vec3 PixiePhysics::GetGravityImpulse(const Rigidbody& body, const float deltaTime)
{
	float mass = 1.0f / body.invertMass;
	return (glm::vec3){0.0f, -GRAVITY, 0.0f} * mass * deltaTime;
}


void PixiePhysics::UpdatePosition(float deltaTime, entt::registry &registry)
{
	auto group = registry.group<Transform, Rigidbody>();
	for (entt::entity entity: group)
	{
		glm::vec3 impulse = GetGravityImpulse(group.get<Rigidbody>(entity), deltaTime);
		ApplyImpulse(group.get<Rigidbody>(entity), impulse);
		
		Transform &transform = group.get<Transform>(entity);
		Rigidbody &body = group.get<Rigidbody>(entity);
		transform.position = transform.position + body.linearVelocity * deltaTime;
		body.linearVelocity.y -= GRAVITY * deltaTime;
	}
}
