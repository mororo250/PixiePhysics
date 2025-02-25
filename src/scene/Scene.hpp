// Dependencies
#include <entt/entt.hpp>
#include "glm/vec3.hpp"
#include <raylib.h>

// Std
#include <vector>
#include <stack>

namespace PixiePhysics
{
    struct PhysicsMaterial
    {
        float friction;
        float elasticity;
        float mass;
    };

    class Scene
    {
    public:
        explicit Scene(entt::registry* registry);
        virtual ~Scene();
        void Update(float dt);
        void CreateSphere(glm::vec3 pos, const PhysicsMaterial& physMaterial, float radius, Color color = WHITE);
        void CreateStaticSphere(glm::vec3 pos, float radius, Color color = WHITE);

        void SaveState();
        void GoBackState();

    private:
        void AddRenderingMaterialComponent(entt::entity entity, Color color) const;

        // Keep up to last 30 frames states
        std::deque<std::vector<entt::any>*> m_states;
        entt::registry*                     m_registry;
    };
}
