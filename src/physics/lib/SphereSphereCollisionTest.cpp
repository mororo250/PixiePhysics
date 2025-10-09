#include "SphereSphereCollisionTest.hpp"

#include "ContactPoint.hpp"

// Components
#include "../components/ShapeLine.hpp"
#include "../components/ShapeSphere.hpp"
#include "../components/RigidBody.hpp"
#include "../components/TransformDynamic.hpp"
#include "../components/Transform.hpp"

namespace PixiePhysics
{
	// Line is point start pos and point end pos
	HasCollided SpherePointSweepTest(const glm::vec3& spherePos,const ShapeSphere &sphere, const ShapeLine &line)
	{
		HasCollided result{};
		result.hasCollided = false;

		const glm::vec3 sphereCenter = spherePos;
		const glm::vec3 lineStart = line.startPos;
		const glm::vec3 lineEnd = line.endPos;

		const glm::vec3 lineDir = lineEnd - lineStart;
		const glm::vec3 toSphere = lineStart - sphereCenter;

		// Solving the equation: a * t^2 + b * t + c = 0
		const float a = dot(lineDir, lineDir);
		const float b = 2.0f * dot(toSphere, lineDir);
		const float c = dot(toSphere, toSphere) - sphere.radius * sphere.radius;

		// Todo: reduce the 3 level of nesting here
		if (const float delta = b * b - 4 * a * c; delta > 0.0f)
		{
			const float sqrtDiscriminant = glm::sqrt(delta);
			const float t1 = (-b - sqrtDiscriminant) / (2.0f * a);
			const float t2 = (-b + sqrtDiscriminant) / (2.0f * a);
			if (t1 >= 0.0f && t1 <= 1.0f)
			{
				result.hasCollided = true;
				result.time = t1;
			}
			else
				if (t2 >= 0.0f && t2 <= 1.0f)
				{
					if (result.hasCollided)
						result.time = (t2 < result.time) ? t2 : result.time;
					else
					{
						result.hasCollided = true;
						result.time = t2;
					}
				}

		}
		else if (delta == 0.0f)
		{
			const float t = -b / (2.0f * a);
			result.time = t;
		}

		return result;
	}

	HasCollided SphereSphereSweepTest(const TransformDynamic& transformA, const TransformDynamic& transformB,
			const ShapeSphere& sphereA, const ShapeSphere& sphereB, const float dt)
	{
		const ShapeSphere sphereAB { sphereA.radius + sphereB.radius };
		const glm::vec3 bodyAVel = transformA.position - transformB.lastPosition;
		const glm::vec3 bodyBVel = transformB.position - transformB.lastPosition;

		const glm::vec3 relativeVelocity = bodyAVel - bodyBVel;
		const glm::vec3 lineStartPos = transformB.lastPosition;
		const glm::vec3 lineEndPos = transformB.lastPosition + relativeVelocity * dt;

		const HasCollided& result = SpherePointSweepTest(transformA.lastPosition, sphereAB,
			ShapeLine{ lineStartPos, lineEndPos});

		return result;
	}

	HasCollided SphereSphereSweepTest(const TransformDynamic& transformA, const Transform& transformB,
		const ShapeSphere& sphereA, const ShapeSphere& sphereB)
	{
		const ShapeSphere sphereAB { sphereA.radius + sphereB.radius };
		const glm::vec3 lineStartPos = transformA.lastPosition;
		const glm::vec3 lineEndPos = transformA.position;

		// Todo: how to deal with when it's already intersecting
		const HasCollided result = SpherePointSweepTest(transformB.position, sphereAB,
			ShapeLine{ lineStartPos, lineEndPos});

		return result;
	}

	bool LineSphereOverlap(const glm::vec3 &spherePos,
					   const ShapeSphere& sphere, const ShapeLine& line)
	{
		const glm::vec3 sphereCenter = spherePos;
		const glm::vec3 lineStart = line.startPos;
		const glm::vec3 lineEnd = line.endPos;

		const glm::vec3 lineDir = lineEnd - lineStart;
		const glm::vec3 toSphere = sphereCenter - lineStart;

		float t = dot(toSphere, lineDir) / dot(lineDir, lineDir);
		t = glm::clamp(t, 0.0f, 1.0f);

		const glm::vec3 closestPoint = lineStart + t * lineDir;
		const float distance = length(closestPoint - sphereCenter);
		return distance <= sphere.radius;
	}

	std::pair<glm::vec3, glm::vec3> GetClosestPointSphereSphere(const glm::vec3 &spherePos, const ShapeSphere &sphere,
		const glm::vec3& spherePos2, const ShapeSphere &sphere2)
	{
		glm::vec3 direction = spherePos2 - spherePos;
		direction = glm::normalize(direction);
		const glm::vec3 closestPoint1 = spherePos + direction * sphere.radius;
		const glm::vec3 closestPoint2 = spherePos2 - direction * sphere2.radius;
		return {closestPoint1, closestPoint2};
	}

	bool SphereSphereOverlap(const TransformDynamic &transformA,
	                         const TransformDynamic &transformB, const ShapeSphere &shapeA,
	                         const ShapeSphere &shapeB)
	{
		glm::vec3 relativePos = transformA.position - transformB.position;
		const float squareDistance = dot(relativePos, relativePos);
		const float sumRad = shapeA.radius + shapeB.radius;
		return squareDistance <= sumRad * sumRad;
	}


}
