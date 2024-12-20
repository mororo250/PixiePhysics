//
// Created by João Mororo on 10/12/2024.
//

#include "ResolveCollisions.hpp"
#include "../ContactPoint.hpp"

namespace PixiePhysics
{
	struct CollisionResult
	{
		bool hasCollided;
		float timeOfImpact; // Time of impact {0 - 1}
		glm::vec3 normal;
		glm::vec3 worldPoint; // Point of collision in world coord
		glm::vec3 localPointA; // Point of collision in local coord of object A
		glm::vec3 localPointB; // Point of collision in local coord oog object B
	};
	
	void ResolveCollision(const ContactPoint &contactPoint, entt::registry &registry);
	
	bool IntersectDiscrete(const Transform &transformA, const Transform &transformB,
	               const SphereShape& shapeA, const SphereShape& shapeB);
	CollisionResult Intersect(const Transform &transformA, const Transform &transformB,
	               const SphereShape& shapeA, const SphereShape& shapeB);
	
}

void PixiePhysics::ResolveCollisions(float deltaTime, entt::registry& registry)
{
	auto entityView = registry.view<entt::entity>();
	std::vector<ContactPoint> contactPoints;
	contactPoints.reserve(entityView.size());

	// Todo: optimization - no need for checking static entities against each other
	// Separate into two loops: dynamic vs dynamic and dynamic vs static
    std::vector entities(entityView.begin(), entityView.end());
	for (std::size_t i = 0; i < entities.size(); i++)
	{
		entt::entity entityA = entities[i];
		Transform &transformA = registry.get<Transform>(entityA);
		SphereShape &sphereShapeA = registry.get<SphereShape>(entityA);
		
		for (std::size_t j = i + 1; j < entities.size(); j++)
		{
			entt::entity entityB = entities[j];
			Transform &transformB = registry.get<Transform>(entityB);
			SphereShape &sphereShapeB = registry.get<SphereShape>(entityB);
			
			if (IntersectDiscrete(transformA, transformB, sphereShapeA, sphereShapeB))
			{
				ContactPoint contactPoint{};
				contactPoint.entityA = entityA;
				contactPoint.entityB = entityB;
				contactPoint.normal = normalize(transformA.position - transformB.position);
				contactPoint.point = contactPoint.normal * sphereShapeA.radius + transformA.position;
				contactPoint.timeOfImpact = deltaTime;
				contactPoint.separationDistance = length(transformA.position - transformB.position) - (sphereShapeA.radius + sphereShapeB.radius);
				
				contactPoints.push_back(contactPoint);
			}
		}
	}
	
	for (const ContactPoint& contactPoint : contactPoints)
	{
		ResolveCollision(contactPoint, registry);
	}
}

// Todo: separate this function into static vs dynamic and dynamic vs dynamic
void PixiePhysics::ResolveCollision(const ContactPoint& contactPoint, entt::registry& registry)
{
	Rigidbody* const bodyA = registry.try_get<Rigidbody>(contactPoint.entityA);
	Rigidbody* const bodyB = registry.try_get<Rigidbody>(contactPoint.entityB);
	
	if (bodyA == nullptr)
	{
		float impulseJ = (1.0f + bodyB->elasticity) * dot(bodyB->linearVelocity, contactPoint.normal) / bodyB->invertMass;
		glm::vec3 impulse = impulseJ * contactPoint.normal;
		ApplyImpulse(*bodyB, -impulse);
		return;
	}
	if (bodyB == nullptr)
	{
		float impulseJ = (1.0f + bodyA->elasticity) * dot(bodyA->linearVelocity, contactPoint.normal) / bodyA->invertMass;
		glm::vec3 impulse = impulseJ * contactPoint.normal;
		ApplyImpulse(*bodyA, impulse);
		return;
	}

	glm::vec3 relativeVelocity = bodyA->linearVelocity - bodyB->linearVelocity;
	float elasticity = bodyA->elasticity * bodyB->elasticity;
	float impulseJ = (1 + elasticity) * dot(relativeVelocity, contactPoint.normal) / (bodyA->invertMass + bodyB->invertMass);
	glm::vec3 impulse = impulseJ * contactPoint.normal;
	ApplyImpulse(*bodyA, impulse);
	ApplyImpulse(*bodyB, -impulse);
	
	/*
	const float ta = bodyA->invertMass / (bodyA->invertMass + bodyB->invertMass);
	const float tb = bodyB->invertMass / (bodyA->invertMass + bodyB->invertMass);
	
	PixiePhysics::Transform &transformA = registry.get<PixiePhysics::Transform>(contactPoint.entityA);
	PixiePhysics::Transform &transformB = registry.get<PixiePhysics::Transform>(contactPoint.entityB);
	transformA.position -= ta * (transformA.position - transformB.position);
	transformB.position += tb * (transformA.position - transformB.position);
	*/
}

void PixiePhysics::ApplyImpulse(Rigidbody& body, const glm::vec3& impulse)
{
	body.linearVelocity += impulse / body.invertMass;
}

// Sphere vs Sphere discrete collision detection
bool PixiePhysics::IntersectDiscrete(const Transform &transformA,
                                     const Transform &transformB, const SphereShape &shapeA,
                                     const SphereShape &shapeB)
{
	// Todo: Use square distance instead of length
	float distance = length(transformA.position - transformB.position);
	float sumRad = shapeA.radius + shapeB.radius;
	return distance <= sumRad;
}

// Sphere vs Sphere sweep collision detection
PixiePhysics::CollisionResult PixiePhysics::Intersect(const Transform& transformA, const Transform& transformB, const SphereShape& shapeA, const SphereShape& shapeB)
{
	CollisionResult result;
	float relativeVelocity;
	
	
}

