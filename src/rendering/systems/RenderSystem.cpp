#include "RenderSystem.hpp"
#include "../../physics/components/SphereShape.hpp"
#include "../../physics/components/Transform.hpp"
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
                    const PixiePhysics::Transform &transform = registry.get<PixiePhysics::Transform>(entity);
                    const PixiePhysics::SphereShape &sphereShape = registry.get<PixiePhysics::SphereShape>(entity);
					const RendererMaterial& material = registry.get<RendererMaterial>(entity);

                    Vector3 position = {transform.position.x, transform.position.y, transform.position.z};
                    DrawSphereEx(position, sphereShape.radius, 64, 64, material.color);
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
