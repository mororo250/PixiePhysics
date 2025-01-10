//
// Created by mororo on 12/24/24.
//

#pragma once
#include <entt/entity/registry.hpp>
#include <glm/vec3.hpp>

namespace PixiePhysics
{
    struct ShapeLine;
    struct TransformStatic;
    struct Rigidbody;
    struct ShapeSphere;
    struct TransformDynamic;
    struct Collision;
    struct HasCollided;

    bool SphereSphereOverlap(const TransformDynamic &transformA, const TransformDynamic &transformB,
                   const ShapeSphere& shapeA, const ShapeSphere& shapeB);

    HasCollided SphereSphereIntersect(const TransformDynamic& transformA, const TransformDynamic& transformB,const ShapeSphere& sphereA,
        const ShapeSphere& sphereB, const Rigidbody& bodyA, const Rigidbody& bodyB, float dt);

    HasCollided SphereSphereIntersect(const TransformDynamic& transformA, const TransformStatic& transformB, const ShapeSphere& sphereA,
        const ShapeSphere& sphereB, const Rigidbody& bodyA);

    bool LineSphereOverlap(const glm::vec3& spherePos, const ShapeSphere& sphere, const ShapeLine& line);
    HasCollided LineSphereIntersect(const glm::vec3& spherePos, const ShapeSphere& sphere, const ShapeLine& line);

}

