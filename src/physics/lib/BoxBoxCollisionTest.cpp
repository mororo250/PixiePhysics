#include "BoxBoxCollisionTest.hpp"

// Components
#include <glm/gtc/quaternion.hpp>

#include "../components/ShapeBox.hpp"
#include "../components/Transform.hpp"


bool PixiePhysics::BoxBoxOverlap(const Transform& transformA, const Transform& transformB,
				  const ShapeBox& shapeA, const ShapeBox& shapeB)
{
	return false;
}

// GJK implementation
std::pair<glm::vec3, glm::vec3> PixiePhysics::GetClosestPointBoxBox(const Transform& transformA, const ShapeBox& box,
	const Transform& transformB, const ShapeBox& box2)
{
	return {glm::vec3(0.0f), glm::vec3(0.0f)}; 
}


// Convert to local space
// Clamp
// Convert back to world space
glm::vec3 PixiePhysics::GetClosestPointBoxPoint(const Transform& transform, const ShapeBox& box, const glm::vec3& point)
{
    glm::quat invRotation = glm::conjugate(transform.rotation);
	glm::vec3 localPoint = invRotation * (point - transform.position);

	glm::vec3 clampedPoint = glm::clamp(localPoint,
		-box.halfExtents,
		box.halfExtents
	);

	return transform.position + transform.rotation * clampedPoint;

}
