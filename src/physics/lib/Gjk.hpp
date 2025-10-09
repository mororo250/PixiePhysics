#pragma once
#include <vector>
#include <glm/vec3.hpp>

#include "BarycentricCoord.hpp"

#include "../components/Transform.hpp"
#include "../components/ShapeSphere.hpp"
#include "../components/ShapeBox.hpp"

#include "../utils/Constatns.hpp"

namespace PixiePhysics
{
	struct SupportPoint
	{
		glm::vec3 point;
		glm::vec3 shapeAPoint;
		glm::vec3 shapeBPoint;
	};

	class Simplex
	{
	public:
		explicit Simplex() { points.reserve(4); }

		void PushFront(const SupportPoint& point)
		{
			points.insert(points.begin(), point);
		}

		void RemoveLast()
		{
			points.pop_back();
		}

		void Remove(int index)
		{
			points.erase(points.begin() + index);
		}

		SupportPoint& operator[](size_t i) { return points[i]; }
		const SupportPoint& operator[](size_t i) const { return points[i]; }
		[[nodiscard]] size_t Size() const { return points.size(); }
		void Clear() { points.clear(); }

	private:
		std::vector<SupportPoint> points;
	};

	// Template for support point calculation based on shape type
	template<typename Shape>
	struct SupportFunction
	{
		static glm::vec3 get(const Shape& shape, const Transform& transform, const glm::vec3& direction) = delete;
	};

    // Forward declare specializations
    template<>
    struct SupportFunction<ShapeSphere>
    {
        static glm::vec3 get(const ShapeSphere& sphere, const Transform& transform, const glm::vec3& direction);
    };

    template<>
    struct SupportFunction<ShapeBox>
    {
        static glm::vec3 get(const ShapeBox& box, const Transform& transform, const glm::vec3& direction);
    };

	enum class EnumLineClosestFeature : unsigned char
	{
		A = 1 << 0,
		B = 1 << 1,
		AB = A | B
	};

	std::pair<glm::vec3, EnumLineClosestFeature> FindClosestPointOnLine(const glm::vec3& a, const glm::vec3& b, const glm::vec3& point);

	// Todo: this is considering the surface an infinity surface
	glm::vec3 FindClosestPointOnSurface(const glm::vec3& vec1, const glm::vec3& vec2, const glm::vec3& point);

	enum class EnumTriangleClosestFeature : unsigned char
	{
		A = 1 << 0,
		B = 1 << 1,
		C = 1 << 2,
		AB = A | B,
		AC = A | C,
		BC = B | C,
		ABC = A | B | C,
	};

	std::pair<glm::vec3, EnumTriangleClosestFeature> FindClosestPointOnTriangle(const glm::vec3& a,
		const glm::vec3& b,
		const glm::vec3& c,
		const glm::vec3& point);

	enum class EnumTetrahedronClosestFeature : unsigned char
	{
		A = 1 << 0,
		B = 1 << 1,
		C = 1 << 2,
		D = 1 << 3,
		AB = A | B,
		AC = A | C,
		AD = A | D,
		BC = B | C,
		BD = B | D,
		CD = C | D,
		ABC = A | B | C,
		ACD = A | C | D,
		ADB = A | D | B,
		BDC = B | D | C,
		ABCD = A | B | C | D, // Inside
	};


	bool IsPointInsideTetrahedron(
		const glm::vec3& a,
		const glm::vec3& b,
		const glm::vec3& c,
		const glm::vec3& d,
		const glm::vec3& point);

	std::pair<glm::vec3, EnumTetrahedronClosestFeature> FindClosestPointOnTetrahedron(
		const glm::vec3& a,
		const glm::vec3& b,
		const glm::vec3& c,
		const glm::vec3& d,
		const glm::vec3& point);

	void CleanSimplex(Simplex& simplex, const unsigned char set);

	bool UpdateSimplex(Simplex& simplex, glm::vec3& dir);

	std::pair<glm::vec3, glm::vec3> CalculatePointAAndB(Simplex& simplex);

	template<typename ShapeA, typename ShapeB>
	std::pair<glm::vec3, glm::vec3> GetClosestPoints(
		const ShapeA& shapeA,
		const Transform& transformA,
		const ShapeB& shapeB,
		const Transform& transformB)
	{
		constexpr float tolerance = EPSILON;

		glm::vec3 dir = transformB.position - transformA.position;
		if (glm::length(dir) < tolerance)
			dir = glm::vec3(1, 0, 0);

		Simplex simplex = Simplex();
		while (true)
		{
			glm::vec3 searchDir = normalize(dir);

			glm::vec3 pointA = SupportFunction<ShapeA>::get(shapeA, transformA, searchDir);
			glm::vec3 pointB = SupportFunction<ShapeB>::get(shapeB, transformB, -searchDir);
			SupportPoint newSupport = {pointA - pointB, pointA, pointB};
			simplex.PushFront(newSupport);

			if (!UpdateSimplex(simplex, dir))
				break;


			// If there are 4 points on the origin we are done
			if (simplex.Size() == 4)
				break;

			if (glm::length(dir) < tolerance)
				break;

		}

		return CalculatePointAAndB(simplex);
	}
}
