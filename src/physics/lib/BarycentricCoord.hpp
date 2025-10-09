#pragma once
#include <glm/geometric.hpp>
#include <glm/vec3.hpp>
#include "../utils/Constatns.hpp"

namespace PixiePhysics
{
	//  An possible optimization of the barycentric coordinates is for several points, v0, v1 and v2 only needs to be computed once
	// Compute barycentric coordinates of closest point to origin for infinite line defined by (inA, inB)
	// where P_projection = u*a + v*b and u + v = 1
	inline std::tuple<bool, float, float> GetBaryCentricCoord(const glm::vec3& lineA, const glm::vec3& lineB, const glm::vec3& point)
	{
		const glm::vec3 localLineA = lineA - point;
		const glm::vec3 localLineB = lineB - point;

		const glm::vec3 ab =  localLineB - localLineA;
		const float distSqr = dot(ab, ab);

		if (distSqr < EPSILON)
		{
			const float lengthASqr = dot(localLineA, localLineA);
			const float lengthBSqr = dot(localLineB, localLineB);
			if (lengthASqr < lengthBSqr)
				return {false, 1.0f, 0.0f};
			else
				return {false, 0.0f, 1.0f};
		}

		float t = dot(-localLineA, ab) / distSqr;
		return {true, 1.0f - t, t};
	}

	// Compute barycentric coordinates of closest point to origin for triangle defined by (triA, triB, tric)
	inline std::tuple<bool, float, float, float> GetBaryCentricCoord(
		const glm::vec3& triA,
		const glm::vec3& tirB,
		const glm::vec3& triC,
		const glm::vec3& point)
	{
		// Taken from: Jolt physics implementations of barycentric coordinates -> https://github.com/jrouwe/JoltPhysics/blob/e7aa192cdce799628e8a196250be955168ee1906/Jolt/Geometry/ClosestPoint.h#L50
		// Their implementation comes from Christer Ericson(3.5 Barycentric coordinate)
		// +ajustment to always use the shortest edge to improvenumerical stability)

		const glm::vec3 localTriA = triA - point;
		const glm::vec3 localTriB = tirB - point;
		const glm::vec3 localTriC = triC - point;

		const glm::vec3 v0 = localTriB - localTriA;
		const glm::vec3 v1 = localTriC - localTriA;
		const glm::vec3 v2 = localTriC - localTriB;

		float d00 = dot(v0, v0);
		float d11 = dot(v1, v1);
		float d22 = dot(v2, v2);

		if (d00 <= d22)
		{
			float d01 = dot(v0, v1);
			float denom = d00 * d11 - d01 * d01;
			// Denominator could also be expressed as the srqLength of cross product (v0 x v1)
			// Denominator must be positive: |v0|^2 * |v1|^2 - (v0 . v1)^2 = |v0|^2 * |v1|^2 * (1 - cos(angle)^2) >= 0
			if (denom > EPSILON)
			{
				// Calculate barycentric coord V ('s' in P = A + V*v0 + W*v1),
				// solving the plane projection equations; numerator is (d01*a1 - d11*a0).
				float a0 = dot(v0, localTriA);
				float a1 = dot(v1, localTriA);
				float v = (d01 * a1 - d11 * a0) / denom;
				float w = (d01 * a0 - d00 * a1) / denom;
				float u = 1.0f - v - w;
				return {true, u, v, w};
			}

			if (d00 > d11)
			{
				auto [isLine, u, v] = GetBaryCentricCoord(localTriA, localTriB, point);
				return {false, u, v, 0};
			}
			else
			{
				auto [isLine, u, w] = GetBaryCentricCoord(localTriA, localTriC, point);
				return {false, u, 0, w};
			}
		}
		else
		{
			float d12 = dot(v1, v2);
			float denom = d11 * d22 - d12 * d12;
			if (denom > EPSILON)
			{
				float c1 = dot(v1, localTriC);
				float c2 = dot(v2, localTriC);
				float u = (d22 * c1 - d12 * c2) / denom;
				float v = (d11 * c2 - d12 * c1) / denom;
				float w = 1.0f - u - v;
				return {true, u, v, w};
			}

			if (d11 > d22)
			{
				auto [isLine, u, w] = GetBaryCentricCoord(localTriA, localTriC, point);
				return {true, u, 0, w};
			}
			else
			{
				auto [isLine, v, w] = GetBaryCentricCoord(localTriB, localTriC, point);
				return {true, 0, v, w};
			}
		}
	}
}
