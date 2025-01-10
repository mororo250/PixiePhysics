#pragma once
#include <vector>
#include <entt/entt.hpp>
#include "glm/vec3.hpp"
#include <raylib.h>

namespace PixiePhysics
{
    struct PhysicsMaterial
    {
        float elasticity;
        float mass;
    };

    class Scene
    {
    public:
        explicit Scene(entt::registry* registry);
        virtual ~Scene();
        void Update(float dt) const;
        void CreateSphere(glm::vec3 pos, const PhysicsMaterial& physMaterial, float radius, Color color = WHITE);
        void CreateStaticSphere(glm::vec3 pos, float radius, Color color = WHITE);

    private:
        void AddRenderingMaterialComponent(entt::entity entity, Color color) const;

        std::vector<entt::entity> m_bodies;
        entt::registry*           m_registry;
    };
}
