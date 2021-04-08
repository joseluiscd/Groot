#include "components.hpp"
#include "resources.hpp"
#include "entity_editor.hpp"
#include <gfx/imgui/imgui_stdlib.h>

void init_components(entt::registry& reg)
{
    auto& entity_editor = reg.ctx<EntityEditor>();

    entity_editor.registerComponent<Name>("Name"); 
}

namespace MM {

template <>
void ComponentEditorWidget<Name>(entt::registry& reg, entt::registry::entity_type e)
{
	auto& t = reg.get<Name>(e);
    ImGui::InputText("Name", &t.name);
}

}