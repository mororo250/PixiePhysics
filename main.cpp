#define GLM_FORCE_RIGHT_HANDED

#include "raylib.h"
#include "raymath.h"

#include "example/Lights.h"

#include <entt/entt.hpp>

#include "src/scene/Scene.hpp"
#include "src/rendering/systems/RenderSystem.hpp"

int main()
{
    constexpr int screenWidth = 800;
    constexpr int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "Pixie Physics");

    Camera camera = { 0 };
    camera.position = { 2.0f, 4.0f, 6.0f };
    camera.target = { 0.0f, 0.5f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Shader shader = LoadShader("assets/shaders/lighting.vs", "assets/shaders/lighting.fs");
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");

    int ambientLoc = GetShaderLocation(shader, "ambient");
    float ambientLight[4] = { 0.1f, 0.1f, 0.1f, 1.0f};
    SetShaderValue(shader, ambientLoc, ambientLight, SHADER_UNIFORM_VEC4);

    const int MAX_LIGHTS = 5;

    Light lights[MAX_LIGHTS] = { 0 };
    lights[0] = CreateLight(LIGHT_DIRECTIONAL, (Vector3){ 0, 50, 0 }, Vector3Zero(), WHITE, shader);

    // Todo: time should not be controlled by ray lib
    SetTargetFPS(60);
    entt::registry registry;
    PixiePhysics::Scene scene = PixiePhysics::Scene(&registry);

    constexpr PixiePhysics::PhysicsMaterial physMaterial { 0.2f, 0.5f, 1.0f };
    scene.CreateSphere(glm::vec3 { -1.0f, 5.0f, 0.0f }, physMaterial, 1.0f, WHITE);
    scene.CreateStaticSphere(glm::vec3 { 0.0f, -999.9, 0.0f }, 1000.0f, WHITE);
    constexpr float dt = 1.0f / 60.0f;

    bool isPaused = false;

    // Todo: Separate physics and rendering loop and make physics take control of ticks time
    while (!WindowShouldClose())
    {
        if (IsKeyPressed(KEY_SPACE))
            isPaused = !isPaused;

        if (!isPaused)
        {
            scene.Update(dt);
        }
        else
        {
            if (IsKeyPressed(KEY_R))
                scene.GoBackState();
            else if (IsKeyPressed(KEY_F))
                scene.Update(dt);
        }

        UpdateCamera(&camera, CAMERA_FREE);
        const float cameraPos[3] = { camera.position.x, camera.position.y, camera.position.z };
        SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);
        PixieRendering::Render(registry, camera, shader);
    }

    CloseWindow();
    return 0;
}
