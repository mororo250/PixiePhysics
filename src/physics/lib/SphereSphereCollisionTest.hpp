#pragma once
#include <vector>

#include <glm/vec3.hpp>

namespace PixiePhysics
{
    struct ShapeLine;
    struct Transform;
    struct Rigidbody;
    struct ShapeSphere;
    struct TransformDynamic;
    struct Collision;
    struct HasCollided;

    HasCollided SpherePointSweepTest(const glm::vec3& spherePos, const ShapeSphere& sphere, const ShapeLine& line);

    HasCollided SphereSphereSweepTest(const TransformDynamic& transformA, const TransformDynamic& transformB,const ShapeSphere& sphereA,
        const ShapeSphere& sphereB, float dt);

    HasCollided SphereSphereSweepTest(const TransformDynamic& transformA, const Transform& transformB, const ShapeSphere& sphereA,
        const ShapeSphere& sphereB);

    bool SphereSphereOverlap(const TransformDynamic &transformA, const TransformDynamic &transformB,
               const ShapeSphere& shapeA, const ShapeSphere& shapeB);
    bool LineSphereOverlap(const glm::vec3& spherePos, const ShapeSphere& sphere, const ShapeLine& line);

    std::pair<glm::vec3, glm::vec3> GetClosestPointSphereSphere(const glm::vec3& spherePos, const ShapeSphere& sphere,
        const glm::vec3& spherePos2, const ShapeSphere& sphere2);

}
