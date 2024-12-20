#include "Scene.hpp"

// Components
#include "../physics/components/SphereShape.hpp"
#include "../physics/components/RigidBody.hpp"
#include "../physics/components/Transform.hpp"
#include "../rendering/components/RendererMaterial.hpp"

// Systems
#include "../physics/systems/ResolveCollisions.hpp"
#include "../physics/systems/UpdateTransform.hpp"

namespace PixiePhysics
{
    Scene::Scene(entt::registry* registry)
        : m_registry(registry)
    {
	}
	
    void Scene::Update(const float deltaTime)
    {
	    ResolveCollisions(deltaTime, *m_registry);
		UpdatePosition(deltaTime, *m_registry);
	}

    void Scene::CreateSphere(glm::vec3 pos, float radius, Color color)
    {
        float mass = 1.0f * radius;
		float elasticity = 1.0f;

        entt::entity entity = m_registry->create();
        m_bodies.push_back(entity);

        m_registry->emplace<SphereShape>(entity, radius);
	    Rigidbody& body = m_registry->emplace<Rigidbody>(entity);
        body.invertMass = mass;
		body.elasticity = elasticity;
		
        Transform& transform = m_registry->emplace<Transform>(entity);
        transform.position = pos;
		m_registry->emplace<PixieRendering::RendererMaterial>(entity, color);
    }
	
	void Scene::CreateStaticSphere(glm::vec3 pos, float radius, Color color)
	{
		entt::entity entity = m_registry->create();
		m_bodies.push_back(entity);
		
		m_registry->emplace<SphereShape>(entity, radius);
		Transform& transform = m_registry->emplace<Transform>(entity);
		transform.position = pos;
		m_registry->emplace<PixieRendering::RendererMaterial>(entity, color);
	}

    Scene::~Scene()
    {
        m_registry->clear();
    }
}