#pragma once

#include "entt/entity/registry.hpp"

namespace PixiePhysics
{
	void UpdatePosition(float dt, entt::registry& registry);
	void UpdateRotation(float dt, entt::registry& registry);
}
