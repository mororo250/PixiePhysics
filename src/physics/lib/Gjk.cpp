#include "Gjk.hpp"

#include <glm/geometric.hpp>
#include <stdexcept>
#include <cassert>

#include "BarycentricCoord.hpp"
namespace PixiePhysics
{
	glm::vec3 SupportFunction<ShapeSphere>::get(const ShapeSphere& sphere, const Transform& transform, const glm::vec3& direction)
	{
		return transform.position + normalize(direction) * sphere.radius;
	}

	glm::vec3 SupportFunction<ShapeBox>::get(const ShapeBox& box, const Transform& transform, const glm::vec3& direction)
	{
		glm::quat invRotation = glm::conjugate(transform.rotation);
		glm::vec3 localDir = invRotation * direction;

		glm::vec3 localSupport(
			(localDir.x >= 0) ? box.halfExtents.x : -box.halfExtents.x,
			(localDir.y >= 0) ? box.halfExtents.y : -box.halfExtents.y,
			(localDir.z >= 0) ? box.halfExtents.z : -box.halfExtents.z
		);

		return transform.position + transform.rotation * localSupport;
	}

	std::pair<glm::vec3, EnumLineClosestFeature> FindClosestPointOnLine(const glm::vec3& a, const glm::vec3& b, const glm::vec3& point)
	{
		glm::vec3 ab = b - a;
		glm::vec3 ap = point - a;

		// if origin is in the voronoi of a pointA
		// O (origin)
		//  \
		//   \
		//    A-------B

		float apDotAb = dot(ab, ap);
		if (apDotAb <= 0)
			return {a, EnumLineClosestFeature::A};

		glm::vec3 ba = a - b;
		glm::vec3 bp = point - b;
		if (dot(ba, bp) <= 0)
			return {b, EnumLineClosestFeature::B};

		float t = apDotAb / glm::dot(ab, ab);
		return {a + t * ab, EnumLineClosestFeature::AB};

	}

	// Todo: this is considering the surface an infinity surface
	glm::vec3 FindClosestPointOnSurface(const glm::vec3& vec1, const glm::vec3& vec2, const glm::vec3& point)
	{
		glm::vec3 normal = glm::normalize(glm::cross(vec1, vec2));
		float distance = glm::dot(point, normal);
		return point - distance * normal;
	}

	std::pair<glm::vec3, EnumTriangleClosestFeature> FindClosestPointOnTriangle(const glm::vec3& a,
		const glm::vec3& b,
		const glm::vec3& c,
		const glm::vec3& point)
	{
		glm::vec3 ab = b - a;
		glm::vec3 ac = c - a;
		glm::vec3 surfaceClosestPoint = FindClosestPointOnSurface(ab, ac, point);

		glm::vec3 bc = c - b;

		glm::vec3 ap = surfaceClosestPoint - a;
		glm::vec3 bp = surfaceClosestPoint - b;
		glm::vec3 cp = surfaceClosestPoint - c;

		glm::vec3 ap_cross_ab = glm::cross(ab, ap);
		glm::vec3 bp_cross_bc = glm::cross(bc, bp);
		glm::vec3 cp_cross_ca = glm::cross(-ac, cp);

		// Ensure point is on the same side of all edges
		if (glm::dot(ap_cross_ab, bp_cross_bc) > 0 && glm::dot(bp_cross_bc, cp_cross_ca) > 0)
		{
			return {surfaceClosestPoint, EnumTriangleClosestFeature::ABC}; // Inside triangle
		}

		// Get the closest point of each line and make return the closest among them
		auto[closestPoint, abFeature] = FindClosestPointOnLine(a, b, point);
		float closestPointDistance = glm::distance(closestPoint, point);

		auto[bcClosestPoint, bcFeature]= FindClosestPointOnLine(b, c, point);
		float otherPointDistance = glm::distance(bcClosestPoint, point);

		EnumTriangleClosestFeature closestFeature;
		if (otherPointDistance < closestPointDistance)
		{
			closestPoint = bcClosestPoint;
			closestPointDistance = otherPointDistance;

			switch (bcFeature)
			{
				case EnumLineClosestFeature::AB:	closestFeature = EnumTriangleClosestFeature::BC;	break;
				case EnumLineClosestFeature::A:		closestFeature = EnumTriangleClosestFeature::B;		break;
				case EnumLineClosestFeature::B:		closestFeature = EnumTriangleClosestFeature::C;		break;
			}
		}
		else
		{
			switch (abFeature)
			{
				case EnumLineClosestFeature::AB:	closestFeature = EnumTriangleClosestFeature::AB;	break;
				case EnumLineClosestFeature::A:		closestFeature = EnumTriangleClosestFeature::A;		break;
				case EnumLineClosestFeature::B:		closestFeature = EnumTriangleClosestFeature::B;		break;
			}
		}

		auto[acClosestPoint, acFeature] = FindClosestPointOnLine(a, c, point);
		otherPointDistance = glm::distance(acClosestPoint, point);

		if (otherPointDistance < closestPointDistance)
		{
			switch (acFeature)
			{
				case EnumLineClosestFeature::AB:
					closestFeature = EnumTriangleClosestFeature::AC;
				break;
				case EnumLineClosestFeature::A:
					closestFeature = EnumTriangleClosestFeature::A;
				break;
				case EnumLineClosestFeature::B:
					closestFeature = EnumTriangleClosestFeature::C;
				break;
			}

			return {acClosestPoint, closestFeature};
		}

		return {closestPoint, closestFeature};
	}

	bool IsPointInsideTetrahedron(
		const glm::vec3& a,
		const glm::vec3& b,
		const glm::vec3& c,
		const glm::vec3& d,
		const glm::vec3& point)
	{
		glm::vec3 normal_abc = glm::cross(b - a, c - a);
		// Ensure normal points outwards (away from d)
		if (glm::dot(normal_abc, d - a) > 0.0f)
			normal_abc = -normal_abc;

		// Check if point is behind or on the plane abc
		if (glm::dot(normal_abc, point - a) > EPSILON)
			return false; // Point is outside face abc

		glm::vec3 normal_acd = glm::cross(c - a, d - a);
		if (glm::dot(normal_acd, b - a) > 0.0f)
			normal_acd = -normal_acd;

		if (glm::dot(normal_acd, point - a) > EPSILON)
			return false;

		glm::vec3 normal_adb = glm::cross(d - a, b - a);
		if (glm::dot(normal_adb, c - a) > 0.0f)
			normal_adb = -normal_adb;

		if (glm::dot(normal_adb, point - a) > EPSILON)
			return false;

		glm::vec3 normal_bdc = glm::cross(d - b, c - b);
		if (glm::dot(normal_bdc, a - b) > 0.0f)
			normal_bdc = -normal_bdc;

		if (glm::dot(normal_bdc, point - b) > EPSILON)
			return false;

		return true;
	}

	std::pair<glm::vec3, EnumTetrahedronClosestFeature> FindClosestPointOnTetrahedron(
		const glm::vec3& a,
		const glm::vec3& b,
		const glm::vec3& c,
		const glm::vec3& d,
		const glm::vec3& point)
	{
		if (IsPointInsideTetrahedron(a, b, c, d, point))
			return {point, EnumTetrahedronClosestFeature::ABCD};

		auto [abcPoint, abcFeature] = FindClosestPointOnTriangle(a, b, c, point);
		auto [acdPoint, acdFeature] = FindClosestPointOnTriangle(a, c, d, point);
		auto [adbPoint, adbFeature] = FindClosestPointOnTriangle(a, d, b, point);
		auto [bdcPoint, bdcFeature] = FindClosestPointOnTriangle(b, d, c, point);

		glm::vec3 closestPoint = abcPoint;
		EnumTetrahedronClosestFeature closestFeature;
		float closestDistance = glm::distance(point, abcPoint);

		switch (abcFeature)
		{
			case EnumTriangleClosestFeature::A:   closestFeature = EnumTetrahedronClosestFeature::A;    break;
			case EnumTriangleClosestFeature::B:   closestFeature = EnumTetrahedronClosestFeature::B;    break;
			case EnumTriangleClosestFeature::C:   closestFeature = EnumTetrahedronClosestFeature::C;    break;
			case EnumTriangleClosestFeature::AB:  closestFeature = EnumTetrahedronClosestFeature::AB;	break;
			case EnumTriangleClosestFeature::AC:  closestFeature = EnumTetrahedronClosestFeature::AC;	break;
			case EnumTriangleClosestFeature::BC:  closestFeature = EnumTetrahedronClosestFeature::BC;	break;
			case EnumTriangleClosestFeature::ABC: closestFeature = EnumTetrahedronClosestFeature::ABC;	break;
		}


		float otherDistance = glm::distance(point, acdPoint);
		if (otherDistance < closestDistance)
		{
			closestPoint = acdPoint;
			closestDistance = otherDistance;
			switch (acdFeature)
			{
				case EnumTriangleClosestFeature::A:     closestFeature = EnumTetrahedronClosestFeature::A;      break;
				case EnumTriangleClosestFeature::B:     closestFeature = EnumTetrahedronClosestFeature::C;      break;
				case EnumTriangleClosestFeature::C:     closestFeature = EnumTetrahedronClosestFeature::D;      break;
				case EnumTriangleClosestFeature::AB:    closestFeature = EnumTetrahedronClosestFeature::AC;     break;
				case EnumTriangleClosestFeature::AC:    closestFeature = EnumTetrahedronClosestFeature::AD;     break;
				case EnumTriangleClosestFeature::BC:    closestFeature = EnumTetrahedronClosestFeature::CD;		break;
				case EnumTriangleClosestFeature::ABC:	closestFeature = EnumTetrahedronClosestFeature::ACD;	break;
			}
		}

		otherDistance = glm::distance(point, adbPoint);
		if (otherDistance < closestDistance)
		{
			closestPoint = adbPoint;
			closestDistance = otherDistance;
			switch (adbFeature)
			{
				case EnumTriangleClosestFeature::A:     closestFeature = EnumTetrahedronClosestFeature::A; break;
				case EnumTriangleClosestFeature::B:     closestFeature = EnumTetrahedronClosestFeature::D; break;
				case EnumTriangleClosestFeature::C:     closestFeature = EnumTetrahedronClosestFeature::B; break;
				case EnumTriangleClosestFeature::AB:    closestFeature = EnumTetrahedronClosestFeature::AD; break;
				case EnumTriangleClosestFeature::AC:    closestFeature = EnumTetrahedronClosestFeature::AB; break;
				case EnumTriangleClosestFeature::BC:    closestFeature = EnumTetrahedronClosestFeature::BD; break;
				case EnumTriangleClosestFeature::ABC:	closestFeature = EnumTetrahedronClosestFeature::ADB; break;
			}
		}

		otherDistance = glm::distance(point, bdcPoint);
		if (otherDistance < closestDistance)
		{
			closestPoint = bdcPoint;
			switch (bdcFeature)
			{
				case EnumTriangleClosestFeature::A:     closestFeature = EnumTetrahedronClosestFeature::B; break;
				case EnumTriangleClosestFeature::B:     closestFeature = EnumTetrahedronClosestFeature::D; break;
				case EnumTriangleClosestFeature::C:     closestFeature = EnumTetrahedronClosestFeature::C; break;
				case EnumTriangleClosestFeature::AB:    closestFeature = EnumTetrahedronClosestFeature::BD; break;
				case EnumTriangleClosestFeature::AC:    closestFeature = EnumTetrahedronClosestFeature::BC; break;
				case EnumTriangleClosestFeature::BC:    closestFeature = EnumTetrahedronClosestFeature::CD; break;
				case EnumTriangleClosestFeature::ABC:	closestFeature = EnumTetrahedronClosestFeature::BDC; break;
			}
		}

		return {closestPoint, closestFeature};
	}

	void CleanSimplex(Simplex& simplex, const unsigned char set)
	{
		for (int i = simplex.Size() - 1; i >= 0 ; --i)
		{
			if ((set & (1 << i)) == 0)
				simplex.Remove(i);
		}
	}

	bool UpdateSimplex(Simplex& simplex, glm::vec3& dir)
	{
		glm::vec3 newDir;
		switch (simplex.Size())
		{
			case 1:
				dir = -simplex[0].point;
				return true;
			case 2:
			{
				auto [closestPoint, feature] = FindClosestPointOnLine(
					simplex[0].point,
					simplex[1].point,
					glm::vec3(0));

				newDir = -closestPoint;
				CleanSimplex(simplex, static_cast<unsigned char>(feature));
				break;
			}
			case 3:
			{
				auto [closestPoint, feature] = FindClosestPointOnTriangle(
					simplex[0].point,
					simplex[1].point,
					simplex[2].point,
					glm::vec3(0));

				newDir = -closestPoint;
				CleanSimplex(simplex, static_cast<unsigned char>(feature));
				break;
			}
			case 4:
			{
				auto [closestPoint, feature] = FindClosestPointOnTetrahedron(simplex[0].point,
					simplex[1].point,
					simplex[2].point,
					simplex[3].point,
					glm::vec3(0));

				if (feature == EnumTetrahedronClosestFeature::ABCD)
					return false;

				newDir = -closestPoint;
				CleanSimplex(simplex, static_cast<unsigned char>(feature));
				break;
			}
			default:
				throw std::runtime_error("Simplex construction failed, Simplex has unexpected number of points");
		}

		float currentDistanceSrq = glm::dot(dir, dir);
		float newLengthSqr = glm::dot(newDir, newDir);
		if (newLengthSqr < currentDistanceSrq)
		{
			dir = newDir;
			return true;
		}
		return false;
	}

	std::pair<glm::vec3, glm::vec3> CalculatePointAAndB(Simplex& simplex)
	{
		switch (simplex.Size())
		{
			case 1:
			 return {simplex[0].shapeAPoint, simplex[0].shapeBPoint};
			case 2:
			{
				auto [isLine, u, v] = GetBaryCentricCoord(simplex[0].point, simplex[1].point, {});
				glm::vec3 outA = u * simplex[0].shapeAPoint + v * simplex[1].shapeAPoint;
				glm::vec3 outB = u * simplex[0].shapeBPoint + v * simplex[1].shapeBPoint;
				return {outA, outB};
			}
			case 3:
			{
				auto [isTriangle ,u, v, w] =
					GetBaryCentricCoord(simplex[0].point, simplex[1].point, simplex[2].point, glm::vec3(0));
				glm::vec3 outA = u * simplex[0].shapeAPoint + v * simplex[1].shapeAPoint + w * simplex[2].shapeAPoint;
				glm::vec3 outB = u * simplex[0].shapeBPoint + v * simplex[1].shapeBPoint + w * simplex[2].shapeBPoint;
				return {outA, outB};
			}
			case 4:
				// EPA (Expanding Polytope Algorithm) is typically used after GJK find penetration depth.
				// For now, we return zero vectors as a placeholder, indicating collision but not providing details.
				return {};
			default:
				assert(false && "CalculatePointAAndB called with invalid simplex size");
		}
	}
}
