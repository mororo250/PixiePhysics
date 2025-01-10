//
// Created by João Mororo on 10/12/2024.
//

#include "ResolveCollisions.hpp"

// std
#include <unordered_set>

// Components
#include "../components/ShapeLine.hpp"
#include "../components/ShapeSphere.hpp"
#include "../components/TransformStatic.hpp"
#include "../components/RigidBody.hpp"
#include "../components/TransformDynamic.hpp"

#include "../lib/SphereIntersectionTest.hpp"
#include "../lib/ContactPoint.hpp"

namespace PixiePhysics
{
	void ResolveCollisionDynamic(const Collision& collision,  const float dt, entt::registry& registry)
	{
		TransformDynamic& transformA = registry.get<TransformDynamic>(collision.entityA);
		TransformDynamic& transformB = registry.get<TransformDynamic>(collision.entityB);
		Rigidbody& bodyA = registry.get<Rigidbody>(collision.entityA);
		Rigidbody& bodyB = registry.get<Rigidbody>(collision.entityB);

		const glm::vec3 collisionAPos = transformA.lastPosition + bodyA.linearVelocity * (collision.time * dt);
		const glm::vec3 collisionBPos = transformB.lastPosition + bodyB.linearVelocity * (collision.time * dt);
		const glm::vec3 normal = glm::normalize(collisionAPos - collisionBPos);

		const glm::vec3 relativeVelocity = bodyA.linearVelocity - bodyB.linearVelocity;
		const float elasticity = bodyA.elasticity * bodyB.elasticity;
		const float impulseJ = (1 + elasticity) * dot(relativeVelocity, normal) / (bodyA.invertMass + bodyB.invertMass);
		const glm::vec3 impulse = impulseJ * normal;

		// UpdateBodies
		ApplyImpulse(bodyA, impulse);
		transformA.position = collisionAPos + bodyA.linearVelocity * (1.0f - collision.time) * dt;

		ApplyImpulse(bodyB, -impulse);
		transformB.position = collisionBPos + bodyB.linearVelocity * (1.0f - collision.time) * dt;
	}

	// collision against static objects
	void ResolveCollisionStatic(const Collision &collision, const float dt, entt::registry &registry)
	{
		const TransformStatic& staticTransform = registry.get<TransformStatic>(collision.entityB);
		TransformDynamic& dynamicTransform = registry.get<TransformDynamic>(collision.entityA);
		Rigidbody& rigidbody = registry.get<Rigidbody>(collision.entityA);

		const glm::vec3 collisionAPos = dynamicTransform.lastPosition + rigidbody.linearVelocity * (collision.time * dt);
		const glm::vec3 normal = glm::normalize(collisionAPos - staticTransform.position);

		const float impulseJ = (1.0f + rigidbody.elasticity) * dot(rigidbody.linearVelocity, normal) / rigidbody.invertMass;
		const glm::vec3 impulse = impulseJ * normal;

		// Update object
		ApplyImpulse(rigidbody, -impulse);
		dynamicTransform.position = collisionAPos + rigidbody.linearVelocity * (1.0f - collision.time) * dt;
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
			Rigidbody& bodyA = registry.get<Rigidbody>(entityA);

			for (std::size_t j = i + 1; j < dynamicGroup.size(); j++)
			{
				const entt::entity entityB = dynamicGroup[j];
				TransformDynamic &transformB = registry.get<TransformDynamic>(entityB);
				ShapeSphere &sphereShapeB = registry.get<ShapeSphere>(entityB);
				Rigidbody& bodyB = registry.get<Rigidbody>(entityB);

				const HasCollided& hasCollided = SphereSphereIntersect(transformA, transformB, sphereShapeA,
					sphereShapeB, bodyA, bodyB, dt);

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
			Rigidbody& bodyA = registry.get<Rigidbody>(entityA);

			for (const entt::entity entityB : staticGroup)
			{
				TransformStatic &transformB = registry.get<TransformStatic>(entityB);
				ShapeSphere &sphereShapeB = registry.get<ShapeSphere>(entityB);

				const HasCollided& hasCollided = SphereSphereIntersect(transformA, transformB, sphereShapeA,
					sphereShapeB, bodyA);

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

	void ApplyImpulse(Rigidbody& body, const glm::vec3& impulse)
	{
		body.linearVelocity += impulse / body.invertMass;
	}
}
