#include "Scene.hpp"

// Components
#include "../physics/components/ShapeSphere.hpp"
#include "../physics/components/RigidBody.hpp"
#include "../physics/components/TransformDynamic.hpp"
#include "../rendering/components/RendererMaterial.hpp"

// Systems
#include "../physics/components/TransformStatic.hpp"
#include "../physics/systems/ResolveCollisions.hpp"
#include "../physics/systems/UpdateTransform.hpp"

namespace PixiePhysics
{
    Scene::Scene(entt::registry* registry)
        : m_registry(registry)
    {
	}
	
    void Scene::Update(const float dt) const
	{
    	UpdatePosition(dt, *m_registry);
	    ResolveCollisions(dt, *m_registry);
	}

    void Scene::CreateSphere(const glm::vec3 pos, const PhysicsMaterial& physMaterial, const float radius, const Color color)
    {
        const entt::entity entity = m_registry->create();
        m_bodies.push_back(entity);

        m_registry->emplace<ShapeSphere>(entity, radius);
	    Rigidbody& body = m_registry->emplace<Rigidbody>(entity);
        body.invertMass = physMaterial.mass;
		body.elasticity = physMaterial.elasticity;
		
        m_registry->emplace<TransformDynamic>(entity, pos);

    	AddRenderingMaterialComponent(entity, color);
    }
	
	void Scene::CreateStaticSphere(const glm::vec3 pos, const float radius, const Color color)
	{
		const entt::entity entity = m_registry->create();
		m_bodies.push_back(entity);

		m_registry->emplace<ShapeSphere>(entity, radius);
		m_registry->emplace<TransformStatic>(entity, pos);

    	AddRenderingMaterialComponent(entity, color);
	}

	void Scene::AddRenderingMaterialComponent(entt::entity entity, Color color) const
	{
    	Model sphereModel = LoadModelFromMesh(GenMeshSphere(1.0f, 64, 64));
    	sphereModel.materials[0].maps[MATERIAL_MAP_ALBEDO].color = WHITE;
    	sphereModel.materials[0].maps[MATERIAL_MAP_METALNESS].value = 0.0f;
    	sphereModel.materials[0].maps[MATERIAL_MAP_ROUGHNESS].value = 0.0f;
    	sphereModel.materials[0].maps[MATERIAL_MAP_OCCLUSION].value = 0.0f;
    	sphereModel.materials[0].maps[MATERIAL_MAP_EMISSION].color = BLACK;

    	sphereModel.materials[0].maps[MATERIAL_MAP_ALBEDO].texture = LoadTexture("assets/resources/uvImageTexture.png");

    	m_registry->emplace<PixieRendering::RendererMaterial>(entity, sphereModel, color);
	}

	Scene::~Scene()
    {
        m_registry->clear();
    }
}