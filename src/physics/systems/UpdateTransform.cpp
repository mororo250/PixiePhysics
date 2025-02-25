#include "UpdateTransform.hpp"

// Systems
#include <iostream>
#include <raymath.h>

#include <glm/ext/quaternion_trigonometric.hpp>

#include "ResolveCollisions.hpp"

#include "../components/RigidBody.hpp"
#include "../components/TransformDynamic.hpp"

const glm::vec3 GRAVITY = {0.0f, -9.81f, 0.0f};

namespace PixiePhysics
{
	glm::vec3 GetGravityImpulse(float dt, const Rigidbody& body);
	glm::mat3 GetInertiaTensor(float mass, float radius);
}

void PixiePhysics::UpdatePosition(const float dt, entt::registry &registry)
{
	auto group = registry.group<TransformDynamic, Rigidbody>();
	for (const entt::entity& entity: group)
	{
		glm::vec3 impulse = GetGravityImpulse(dt, group.get<Rigidbody>(entity));
		ApplyImpulseLinear(group.get<Rigidbody>(entity), impulse);
		
		TransformDynamic &transform = group.get<TransformDynamic>(entity);
		Rigidbody &body = group.get<Rigidbody>(entity);
		transform.lastPosition = transform.position;

		// Velocity needs to be updated before position(because of colision restitution)
		// Can't compute acceleration into euler formula because it transforms the movement into a curve and highly complicates the sweep test
		body.linearVelocity += GRAVITY * dt;
		transform.position += body.linearVelocity * dt;
	}
}

glm::vec3 PixiePhysics::GetGravityImpulse(const float dt, const Rigidbody& body)
{
	float mass = 1.0f / body.invertMass;
	return GRAVITY* mass * dt;
}

void PixiePhysics::UpdateRotation(const float dt, entt::registry &registry)
{
	auto group = registry.group<TransformDynamic, Rigidbody>();
	for (const entt::entity& entity: group)
	{
		TransformDynamic &transform = group.get<TransformDynamic>(entity);
		Rigidbody &body = group.get<Rigidbody>(entity);

		float angle = glm::length(body.angularVelocity) * dt;
		if (angle < EPSILON)
			continue;
		glm::vec3 axis = glm::normalize(body.angularVelocity);
		glm::quat angularDif = glm::angleAxis(angle, axis);

		transform.lastRotation = transform.rotation;
		transform.rotation = angularDif * transform.rotation;

		// Normalize the quaternion to avoid floating point errors
		transform.rotation = glm::normalize(transform.rotation);
	}
}
