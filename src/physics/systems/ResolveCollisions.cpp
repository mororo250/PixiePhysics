//
// Created by João Mororo on 10/12/2024.
//

#include "ResolveCollisions.hpp"

// std
#include <unordered_set>

// Components
#include <raymath.h>

#include <glm/ext/quaternion_trigonometric.hpp>

#include "../components/ShapeLine.hpp"
#include "../components/ShapeSphere.hpp"
#include "../components/TransformStatic.hpp"
#include "../components/RigidBody.hpp"
#include "../components/StaticBody.hpp"
#include "../components/TransformDynamic.hpp"

#include "../lib/SphereSphereCollisionTest.hpp"
#include "../lib/ContactPoint.hpp"

namespace PixiePhysics
{
	void ResolveCollisionDynamic(const Collision& collision,  const float dt, entt::registry& registry)
	{
		TransformDynamic& transformA = registry.get<TransformDynamic>(collision.entityA);
		TransformDynamic& transformB = registry.get<TransformDynamic>(collision.entityB);
		Rigidbody& bodyA = registry.get<Rigidbody>(collision.entityA);
		Rigidbody& bodyB = registry.get<Rigidbody>(collision.entityB);
		ShapeSphere& sphereShapeA = registry.get<ShapeSphere>(collision.entityA);

		const glm::vec3 collisionAPos = transformA.lastPosition + bodyA.linearVelocity * (collision.time * dt);
		const glm::vec3 collisionBPos = transformB.lastPosition + bodyB.linearVelocity * (collision.time * dt);
		const glm::vec3 normal = glm::normalize(collisionAPos - collisionBPos);
		const glm::vec3 collisionPoint = collisionAPos + normal * sphereShapeA.radius;

		const glm::vec3 relativeVelocity = bodyA.linearVelocity - bodyB.linearVelocity;
		const float elasticity = bodyA.elasticity * bodyB.elasticity;
		const float impulseJ = (1 + elasticity) * dot(relativeVelocity, normal) / (bodyA.invertMass + bodyB.invertMass);
		const glm::vec3 impulse = impulseJ * normal;

		glm::vec3 localACollisionPoint = collisionPoint - collisionAPos;
		ApplyImpulse(bodyA, impulse, localACollisionPoint);
		glm::vec3 localBCollisionPoint = collisionPoint - collisionBPos;
		ApplyImpulse(bodyB, -impulse, localBCollisionPoint);

		transformA.position = collisionAPos + bodyA.linearVelocity * (1.0f - collision.time) * dt;
		transformB.position = collisionBPos + bodyB.linearVelocity * (1.0f - collision.time) * dt;
	}

	void ResolveCollisionStatic(const Collision &collision, const float dt, entt::registry &registry)
	{
		const TransformStatic& staticTransform = registry.get<TransformStatic>(collision.entityB);
		const StaticBody& staticBody = registry.get<StaticBody>(collision.entityB);
		TransformDynamic& dynamicTransform = registry.get<TransformDynamic>(collision.entityA);
		Rigidbody& rigidbody = registry.get<Rigidbody>(collision.entityA);
		ShapeSphere& sphereShapeA = registry.get<ShapeSphere>(collision.entityA);

		// Calculate position and rotation at the point of collision
		const glm::vec3 collisionAPos = dynamicTransform.lastPosition + rigidbody.linearVelocity * (collision.time * dt);
		glm::quat collisionARotation = dynamicTransform.lastRotation;
		float angle = glm::length(rigidbody.angularVelocity) * (collision.time * dt);
		if (angle > EPSILON)
		{
			const glm::vec3 axis = glm::normalize(rigidbody.angularVelocity);
			const glm::quat angularDif = glm::angleAxis(angle, axis);
			collisionARotation = glm::normalize(angularDif * dynamicTransform.lastRotation);
		}

		const glm::vec3 normal = glm::normalize(collisionAPos - staticTransform.position);
		const glm::vec3 collisionPoint = collisionAPos + normal * sphereShapeA.radius;
		glm::vec3 localColPoint = collisionPoint - collisionAPos;
		glm::vec3 vel = rigidbody.linearVelocity + glm::cross(rigidbody.angularVelocity, localColPoint);

		float elasticity = rigidbody.elasticity * staticBody.elasticity;
		// Todo: why is angular factor and how why add it here in the denominator?
		const float impulseJ = (1.0f + elasticity) * dot(vel, normal) / rigidbody.invertMass;
		const glm::vec3 impulse = impulseJ * normal;
		ApplyImpulse(rigidbody, -impulse, localColPoint);
		ApplyFriction(rigidbody, staticBody, normal, localColPoint, vel);

		dynamicTransform.position = collisionAPos + rigidbody.linearVelocity * (1.0f - collision.time) * dt;

		// Compute new rotation
		angle = glm::length(rigidbody.angularVelocity) * (1.0f - collision.time) * dt;
		if (angle < EPSILON)
			return;

		const glm::vec3 axis = glm::normalize(rigidbody.angularVelocity);
		const glm::quat angularDif = glm::angleAxis(angle, axis);
		dynamicTransform.rotation = glm::normalize(angularDif * collisionARotation);
	}

	void ApplyFriction(Rigidbody& rigidbody, const StaticBody& staticBody, const glm::vec3& normal, const glm::vec3& localColPoint, const glm::vec3& vel)
	{
		const float friction = rigidbody.friction * staticBody.friction;
		if (friction <= EPSILON)
			return;

		const float velNormal = dot(vel, normal);
		const glm::vec3 velTangent = vel - normal * velNormal;
		if (glm::length(velTangent) <= EPSILON)
			return;

		const glm::vec3 tanDir = glm::normalize(velTangent);
		const glm::vec3 inertia = glm::cross(rigidbody.inverseInertiaTensor * glm::cross(localColPoint, tanDir), localColPoint);
		const float invInertia = glm::dot(inertia, tanDir);
		const float reducedMass = 1.0f / (rigidbody.invertMass + invInertia);

		const glm::vec3 frictionImpulse = velTangent * friction * reducedMass;
		ApplyImpulse(rigidbody, -frictionImpulse, localColPoint);
	}

	// Todo: Split this function into smaller functions
	void ResolveCollisions(const float dt, entt::registry& registry)
	{
		// Todo: avoid reallocation of collision and firstCollision vectors every frame
		std::vector<Collision> collisions;
		const auto dynamicGroup = registry.group<TransformDynamic, Rigidbody>();
		collisions.reserve(dynamicGroup.size() * dynamicGroup.size());
		for (std::size_t i = 0; i < dynamicGroup.size(); i++)
		{
			const entt::entity entityA = dynamicGroup[i];
			TransformDynamic &transformA = dynamicGroup.get<TransformDynamic>(entityA);
			ShapeSphere &sphereShapeA = registry.get<ShapeSphere>(entityA);

			for (std::size_t j = i + 1; j < dynamicGroup.size(); j++)
			{
				const entt::entity entityB = dynamicGroup[j];
				TransformDynamic &transformB = registry.get<TransformDynamic>(entityB);
				ShapeSphere &sphereShapeB = registry.get<ShapeSphere>(entityB);

				const HasCollided& hasCollided = SphereSphereIntersect(transformA, transformB, sphereShapeA,
					sphereShapeB, dt);

				if (hasCollided.hasCollided)
				{
					Collision collision{entityA, entityB, false, hasCollided.time};
					collisions.push_back(collision);
				}
			}
		}

		const auto staticGroup = registry.view<TransformStatic>();
		for (const entt::entity entityA : dynamicGroup)
		{
			TransformDynamic &transformA = registry.get<TransformDynamic>(entityA);
			ShapeSphere &sphereShapeA = registry.get<ShapeSphere>(entityA);

			for (const entt::entity entityB : staticGroup)
			{
				TransformStatic &transformB = registry.get<TransformStatic>(entityB);
				ShapeSphere &sphereShapeB = registry.get<ShapeSphere>(entityB);

				const HasCollided& hasCollided = SphereSphereIntersect(transformA, transformB, sphereShapeA,
					sphereShapeB);

				if (hasCollided.hasCollided)
				{
					Collision collision{entityA, entityB, true, hasCollided.time};
					collisions.push_back(collision);
				}
			}
		}

		std::ranges::sort(collisions.begin(), collisions.end(),
			[](const Collision& a, const Collision& b) {return a.time < b.time;});

		std::unordered_set<entt::entity> collidedEntities;
		std::vector<Collision> filteredCollisions;
		filteredCollisions.reserve(collisions.size());
		for (const Collision& collision: collisions)
		{
			if (collidedEntities.contains(collision.entityA) || collidedEntities.contains(collision.entityB))
				continue;

			filteredCollisions.push_back(collision);
			collidedEntities.insert(collision.entityA);
			collidedEntities.insert(collision.entityB);
		}

		for (const Collision& collision: filteredCollisions)
		{
			if (collision.isStatic)
				ResolveCollisionStatic(collision, dt, registry);
			else
				ResolveCollisionDynamic(collision, dt, registry);
		}
	}

	void ApplyImpulse(Rigidbody& body, const glm::vec3& impulse, const glm::vec3& impulsePoint)
	{
		ApplyImpulseLinear(body, impulse);
		ApplyImpulseAngular(body, impulse, impulsePoint);
	}

	void ApplyImpulseLinear(Rigidbody& body, const glm::vec3& impulse)
	{
		body.linearVelocity += impulse / body.invertMass;
	}

	// impulse point is in relative posiution
	void ApplyImpulseAngular(Rigidbody& body, const glm::vec3& impulse, const glm::vec3& impulsePoint)
	{
		glm::vec3 dir = impulsePoint - body.centerOfMass;
		glm::vec3 torque = glm::cross(dir, impulse);
		body.angularVelocity += body.inverseInertiaTensor * torque;
	}
}
