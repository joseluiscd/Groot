#include "components.hpp"
#include "resources.hpp"
#include "entity_editor.hpp"
#include <gfx/imgui/imgui_stdlib.h>

void init_components(entt::registry& reg)
{
    auto& entity_editor = reg.ctx<EntityEditor>();

    entity_editor.registerComponent<Name>("Name"); 
    entity_editor.registerComponent<Selected>("Selected");

    // Make only one selected item
    reg.on_construct<Selected>().before<&entt::registry::clear<Selected>>();
}

namespace MM {

template <>
void ComponentEditorWidget<Name>(entt::registry& reg, entt::registry::entity_type e)
{
	auto& t = reg.get<Name>(e);
    ImGui::InputText("Name", &t.name);
}

}