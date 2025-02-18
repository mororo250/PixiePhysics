#include "RenderSystem.hpp"

#include <glm/ext/quaternion_trigonometric.hpp>

#include "../../physics/components/ShapeSphere.hpp"
#include "../../physics/components/TransformDynamic.hpp"
#include "../../physics/components/TransformStatic.hpp"
#include "../../rendering/components/RendererMaterial.hpp"

void PixieRendering::Render(entt::registry& registry, const Camera& camera, const Shader& shader)
{
    BeginDrawing();
    {
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);
        {
            // Not using shader for now because it's not working with spheres
            //BeginShaderMode(shader);
            {
                for (entt::entity entity : registry.view<entt::entity>())
				{
                    glm::vec3 pos;
                    glm::vec3 axisRot;
                    float angleRotation;
                    if (PixiePhysics::TransformDynamic* transform =
                        registry.try_get<PixiePhysics::TransformDynamic>(entity);
                        transform != nullptr)
                    {
                        pos = transform->position;
                        axisRot = glm::axis(transform->rotation);
                        angleRotation = glm::angle(transform->rotation);
                    }
                    else if (PixiePhysics::TransformStatic* transformStatic =
                        registry.try_get<PixiePhysics::TransformStatic>(entity);
                        transformStatic != nullptr)
                    {
                        pos = transformStatic->position;
                        axisRot = glm::axis(transformStatic->rotation);
                        angleRotation = glm::angle(transformStatic->rotation);
                    }
                    else continue;

                    const PixiePhysics::ShapeSphere& sphereShape = registry.get<PixiePhysics::ShapeSphere>(entity);
					const RendererMaterial& material = registry.get<RendererMaterial>(entity);

                    const Vector3 position = {pos.x, pos.y, pos.z};
                    const Vector3 scale = {sphereShape.radius, sphereShape.radius, sphereShape.radius};
                    const Vector3 axisRotation = {axisRot.x, axisRot.y, axisRot.z};
                    DrawModelEx(material.model, position, axisRotation, glm::degrees(angleRotation), scale, material.color );
                }
			}
            //EndShaderMode();

            DrawGrid(100, 1.0f);
        }
        EndMode3D();

        DrawFPS(10, 10);
    }
    EndDrawing();
}
