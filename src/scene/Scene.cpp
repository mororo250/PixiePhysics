#include "Scene.hpp"

// Components
#include "../physics/components/ShapeSphere.hpp"
#include "../physics/components/RigidBody.hpp"
#include "../physics/components/TransformDynamic.hpp"
#include "../rendering/components/RendererMaterial.hpp"

// Systems
#include <iostream>

#include "../physics/components/StaticBody.hpp"
#include "../physics/components/TransformStatic.hpp"
#include "../physics/systems/ResolveCollisions.hpp"
#include "../physics/systems/UpdateTransform.hpp"

namespace PixiePhysics
{
    Scene::Scene(entt::registry* registry)
        : m_registry(registry)
    {
	}
	
    void Scene::Update(const float dt)
	{
    	UpdatePosition(dt, *m_registry);
    	UpdateRotation(dt, *m_registry);
	    ResolveCollisions(dt, *m_registry);

    	if (m_states.size() == 30)
		{
    		std::vector<entt::any>* dataPtr = m_states.back();
			m_states.pop_back();
    		delete dataPtr;
		}

    	SaveState();
	}

    void Scene::CreateSphere(const glm::vec3 pos, const PhysicsMaterial& physMaterial, const float radius, const Color color)
    {
    	glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        const entt::entity entity = m_registry->create();

        m_registry->emplace<ShapeSphere>(entity, radius);
	    Rigidbody& body = m_registry->emplace<Rigidbody>(entity);
        body.invertMass = 1 / physMaterial.mass;
		body.elasticity = physMaterial.elasticity;
    	body.friction = physMaterial.friction;

    	// Calculate inertia tensor for sphere with constant density
    	const float it = 2.5f * body.invertMass / (radius * radius);
    	body.inverseInertiaTensor = glm::mat3(
    		it, 0, 0,
    		0, it, 0,
    		0, 0, it);

        m_registry->emplace<TransformDynamic>(entity, pos, rotation, pos, rotation);
    	AddRenderingMaterialComponent(entity, color);
    }
	
	void Scene::CreateStaticSphere(const glm::vec3 pos, const float radius, const Color color)
	{
    	glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
		const entt::entity entity = m_registry->create();

		m_registry->emplace<ShapeSphere>(entity, radius);
		m_registry->emplace<TransformStatic>(entity, pos, rotation);
    	m_registry->emplace<StaticBody>(entity, 1.0f, 1.0f);
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

	void Scene::SaveState()
    {
    	std::vector<entt::any>* data = new std::vector<entt::any>{};
    	std::function<void(entt::any)> archive =
			[&data]<typename T0>(T0 &&elem) { data->emplace_back(std::forward<T0>(elem)); };

    	entt::snapshot snapshot(*m_registry);
    	snapshot.get<entt::entity>(archive)
    		.get<PixiePhysics::ShapeSphere>(archive)
    		.get<PixiePhysics::Rigidbody>(archive)
    		.get<PixiePhysics::StaticBody>(archive)
    		.get<PixiePhysics::TransformDynamic>(archive)
    		.get<PixiePhysics::TransformStatic>(archive)
    		.get<PixieRendering::RendererMaterial>(archive);
    	m_states.push_front(data);
    }

	void Scene::GoBackState()
    {
    	if (m_states.empty())
    		return;

    	m_registry->clear();
    	std::vector<entt::any>* dataPtr = m_states.front();

    	auto archive = [dataPtr, pos = 0u]<typename T0>(T0 &elem) mutable
    	{
    		elem = entt::any_cast<std::remove_reference_t<T0>>((*dataPtr)[pos++]);
    	};

    	// Todo: can this code be generated so we don't need to rewrite it every time I add a new component?
    	entt::snapshot_loader(*m_registry).get<entt::entity>(archive)
    		.get<PixiePhysics::ShapeSphere>(archive)
			.get<PixiePhysics::Rigidbody>(archive)
    		.get<PixiePhysics::StaticBody>(archive)
			.get<PixiePhysics::TransformDynamic>(archive)
			.get<PixiePhysics::TransformStatic>(archive)
			.get<PixieRendering::RendererMaterial>(archive);

    	m_states.pop_front();
    	delete dataPtr;
    }

	Scene::~Scene()
    {
    	for (std::vector<entt::any>* state : m_states)
    		delete state;

    	m_states.clear();
        m_registry->clear();
    }
}