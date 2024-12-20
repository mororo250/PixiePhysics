#pragma once
#include <vector>
#include <entt/entt.hpp>
#include "glm/vec3.hpp"
#include <raylib.h>

namespace PixiePhysics
{
    class Scene
    {
    public:
        explicit Scene(entt::registry* registry);
        virtual ~Scene();
        void Update(float deltaTime);
        void CreateSphere(glm::vec3 pos, float radius, Color color);
        void CreateStaticSphere(glm::vec3 pos, float radius, Color color);

    private:
        std::vector<entt::entity> m_bodies;
        entt::registry*           m_registry;
    };
}
