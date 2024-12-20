#pragma once

#include "entt/entity/registry.hpp"
#include "raylib.h"

namespace PixieRendering
{
    void Render(entt::registry& registry, const Camera& camera, const Shader& shader);
}
