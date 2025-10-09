#pragma once
#include <vector>
#include <glm/vec3.hpp>

namespace PixiePhysics
{
	struct Transform;
	struct ShapeLine;
	struct Rigidbody;
	struct ShapeBox;
	struct TransformDynamic;
	struct Collision;
	struct HasCollided;

	bool BoxBoxOverlap(const Transform &transformA, const Transform &transformB,
			   const ShapeBox& shapeA, const ShapeBox& shapeB);

	std::pair<glm::vec3, glm::vec3> GetClosestPointBoxBox(const Transform& transformA, const ShapeBox& box,
		const Transform& transformB, const ShapeBox& box2);

	glm::vec3 GetClosestPointBoxPoint(const Transform& transform, const ShapeBox& box, const glm::vec3& point);

	glm::vec3 SupportFunctionBox(const glm::vec3& direction, const ShapeBox& box);
}
