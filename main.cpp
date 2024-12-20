// raylib
#include "raylib.h"
#include "raymath.h"

#include "example/Lights.h"

#include <glm/vec2.hpp>
#include <entt/entt.hpp>

#include "src/scene/Scene.hpp"
#include "src/rendering/systems/RenderSystem.hpp"

#define GLSL_VERSION            330

int main()
{
    glm::ivec2 a = { 1, 2 };

    const int screenWidth = 800;
    const int screenHeight = 450;
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
    SetShaderValue(shader, ambientLoc, (float[4]){ 0.1f, 0.1f, 0.1f, 1.0f }, SHADER_UNIFORM_VEC4);

    // Create lights

    const int MAX_LIGHTS = 5;

    // Todo: this shader is not working with spheres for some reason?
    // Todo: Write simple shader to fix this
    Light lights[MAX_LIGHTS] = { 0 };
    lights[0] = CreateLight(LIGHT_DIRECTIONAL, (Vector3){ 0, 50, 0 }, Vector3Zero(), WHITE, shader);

    SetTargetFPS(60);
    entt::registry registry;
    PixiePhysics::Scene scene = PixiePhysics::Scene(&registry);
    scene.CreateSphere(glm::vec3 { 0.0f, 5.0f, 0.0f }, 1.0f, RED);
    scene.CreateStaticSphere(glm::vec3 { 0.0f, -999.9, 0.0f }, 1000.0f, WHITE);
    const float deltaTime = 1.0f / 60.0f;

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_ORBITAL);

        float cameraPos[3] = { camera.position.x, camera.position.y, camera.position.z };
        SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);
        PixieRendering::Render(registry, camera, shader);
        scene.Update(deltaTime);
    }

    CloseWindow();
    return 0;
}
