// for the license, see the end of the file
#pragma once

#include <functional>
#include <map>
#include <set>
#include <string>

#include "components.hpp"
#include "entt.hpp"
#include <gfx/imgui/imgui.h>

#ifndef MM_IEEE_ASSERT
#define MM_IEEE_ASSERT(x) assert(x)
#endif

#define MM_IEEE_IMGUI_PAYLOAD_TYPE_ENTITY "MM_IEEE_ENTITY"

#ifndef MM_IEEE_ENTITY_WIDGET
#define MM_IEEE_ENTITY_WIDGET ::MM::EntityWidget
#endif

namespace MM {

template <class EntityType>
inline void EntityWidget(EntityType& e, entt::basic_registry<EntityType>& reg, bool dropTarget = false)
{
    ImGui::PushID(static_cast<int>(entt::to_integral(e)));

    if (reg.valid(e)) {
		const char * name;
		char namebuff[32];
		auto& selected = reg.template ctx<SelectedEntity>().selected;
        bool visible = reg.template all_of<Visible>(e);

        if (reg.template all_of<Name>(e)) {
			name = reg.template get<Name>(e).name.c_str();
        } else {
			sprintf(namebuff, "Entity %x", entt::to_integral(e));
			name = namebuff;
        }

        ImGui::PushID("###Checkbox");
        ImGui::PushID((void*) e);
        if (ImGui::Checkbox("###CB", &visible)) {
            if (visible) {
                reg.template emplace<Visible>(e);
            } else {
                reg.template remove<Visible>(e);
            }
        }
        ImGui::PopID();
        ImGui::PopID();

        ImGui::SameLine();
		if (ImGui::Selectable(name, selected == e)){
			selected = e;
		}

    } else {
        ImGui::Text("Invalid Entity");
    }

    ImGui::PopID();
}

template <class Component, class EntityType>
void ComponentEditorWidget([[maybe_unused]] entt::basic_registry<EntityType>& registry, [[maybe_unused]] EntityType entity) { }

template <class Component, class EntityType>
void ComponentAddAction(entt::basic_registry<EntityType>& registry, EntityType entity)
{
    registry.template emplace<Component>(entity);
}

template <class Component, class EntityType>
void ComponentRemoveAction(entt::basic_registry<EntityType>& registry, EntityType entity)
{
    registry.template remove<Component>(entity);
}

template <class EntityType>
class EntityEditor {
public:
    using Registry = entt::basic_registry<EntityType>;
    using ComponentTypeID = entt::id_type;

    struct ComponentInfo {
        using Callback = std::function<void(Registry&, EntityType)>;
        std::string name;
        Callback widget, create, destroy;
        bool alt_color;
    };

    bool show_window = true;

private:
    std::map<ComponentTypeID, ComponentInfo> component_infos;

    bool entityHasComponent(Registry& registry, EntityType& entity, ComponentTypeID type_id)
    {
        ComponentTypeID type[] = { type_id };
        return registry.runtime_view(std::cbegin(type), std::cend(type)).contains(entity);
    }

public:
    template <class Component>
    ComponentInfo& registerComponent(const ComponentInfo& component_info)
    {
        auto index = entt::type_hash<Component>::value();
        auto insert_info = component_infos.insert_or_assign(index, component_info);
        MM_IEEE_ASSERT(insert_info.second);
        return std::get<ComponentInfo>(*insert_info.first);
    }

    template <class Component>
    ComponentInfo& registerComponent(const std::string& name, typename ComponentInfo::Callback widget, bool alt_color = false)
    {
        return registerComponent<Component>(ComponentInfo {
            name,
            widget,
            ComponentAddAction<Component, EntityType>,
            ComponentRemoveAction<Component, EntityType>,
            alt_color
        });
    }

    template <class Component>
    ComponentInfo& registerComponent(const std::string& name, bool alt_color = false)
    {
        return registerComponent<Component>(name, ComponentEditorWidget<Component, EntityType>, alt_color);
    }

    void renderEditor(Registry& registry, EntityType& e, bool disable_delete = false)
    {
        ImGui::TextUnformatted("Editing:");
        ImGui::SameLine();

        MM_IEEE_ENTITY_WIDGET(e, registry, true);

        if (ImGui::Button("New")) {
            e = registry.create();
        }
        if (registry.valid(e)) {
            ImGui::SameLine();

            // clone would go here
            //if (ImGui::Button("Clone")) {
            //auto old_e = e;
            //e = registry.create();
            //}

            ImGui::Dummy({ 10, 0 }); // space destroy a bit, to not accidentally click it
            ImGui::SameLine();

            // red button
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.65f, 0.15f, 0.15f, 1.f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.3f, 0.3f, 1.f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1.f, 0.2f, 0.2f, 1.f));
            if (!disable_delete && ImGui::Button("Destroy")) {
                registry.destroy(e);
                e = entt::null;
            }
            ImGui::PopStyleColor(3);
        }

        ImGui::Separator();

        if (registry.valid(e)) {
            ImGuiStyle& style = ImGui::GetStyle();

            ImGui::PushID(static_cast<int>(entt::to_integral(e)));
            std::map<ComponentTypeID, ComponentInfo> has_not;
            for (auto& [component_type_id, ci] : component_infos) {
                if (entityHasComponent(registry, e, component_type_id)) {
                    ImGui::PushID(component_type_id);
                    if (!disable_delete && ImGui::Button("-")) {
                        ci.destroy(registry, e);
                        ImGui::PopID();
                        continue; // early out to prevent access to deleted data
                    } else if (!disable_delete){
                        ImGui::SameLine();
                    }

                    if (ci.alt_color) {
                        ImGui::PushStyleColor(ImGuiCol_Header, style.Colors[ImGuiCol_PlotHistogram]);
                        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, style.Colors[ImGuiCol_PlotHistogramHovered]);
                        ImGui::PushStyleColor(ImGuiCol_HeaderActive, style.Colors[ImGuiCol_PlotHistogram]);
                    }
                    if (ImGui::CollapsingHeader(ci.name.c_str())) {
                        ImGui::Indent(30.f);
                        ImGui::PushID("Widget");
                        ci.widget(registry, e);
                        ImGui::PopID();
                        ImGui::Unindent(30.f);
                    }
                    if (ci.alt_color) {
                        ImGui::PopStyleColor(3);
                    }

                    ImGui::PopID();
                } else {
                    has_not[component_type_id] = ci;
                }
            }

            if (!has_not.empty()) {
                if (ImGui::Button("+ Add Component")) {
                    ImGui::OpenPopup("Add Component");
                }

                if (ImGui::BeginPopup("Add Component")) {
                    ImGui::TextUnformatted("Available:");
                    ImGui::Separator();

                    for (auto& [component_type_id, ci] : has_not) {
                        ImGui::PushID(component_type_id);
                        if (ImGui::Selectable(ci.name.c_str())) {
                            ci.create(registry, e);
                        }
                        ImGui::PopID();
                    }
                    ImGui::EndPopup();
                }
            }
            ImGui::PopID();
        }
    }

    void renderEntityList(Registry& registry, std::set<ComponentTypeID>& comp_list)
    {
        ImGui::Text("Components Filter:");
        ImGui::SameLine();
        if (ImGui::SmallButton("clear")) {
            comp_list.clear();
        }

        ImGui::Indent();

        for (const auto& [component_type_id, ci] : component_infos) {
            bool is_in_list = comp_list.count(component_type_id);
            bool active = is_in_list;

            ImGui::Checkbox(ci.name.c_str(), &active);

            if (is_in_list && !active) { // remove
                comp_list.erase(component_type_id);
            } else if (!is_in_list && active) { // add
                comp_list.emplace(component_type_id);
            }
        }

        ImGui::Unindent();
        ImGui::Separator();

        if (comp_list.empty()) {
            ImGui::Text("All entities:");
            registry.each([&registry](auto e) {
                MM_IEEE_ENTITY_WIDGET(e, registry, false);
            });
        } else {
            auto view = registry.runtime_view(comp_list.begin(), comp_list.end());
            ImGui::Text("%lu Entities Matching:", view.size_hint());

            if (ImGui::BeginChild("entity list")) {
                for (auto e : view) {
                    MM_IEEE_ENTITY_WIDGET(e, registry, false);
                }
            }
            ImGui::EndChild();
        }
    }

    [[deprecated("Use renderEditor() instead. And manage the window yourself.")]] void render(Registry& registry, EntityType& e)
    {
        if (show_window) {
            if (ImGui::Begin("Entity Editor", &show_window)) {
                renderEditor(registry, e);
            }
            ImGui::End();
        }
    }

    // displays both, editor and list
    // uses static internally, use only as a quick way to get going!
    void renderSimpleCombo(Registry& registry, EntityType& e)
    {
        if (show_window) {
            ImGui::SetNextWindowSize(ImVec2(550, 400), ImGuiCond_FirstUseEver);
            if (ImGui::Begin("Entity Editor", &show_window)) {
                if (ImGui::BeginChild("list", { 200, 0 }, true)) {
                    static std::set<ComponentTypeID> comp_list;
                    renderEntityList(registry, comp_list);
                }
                ImGui::EndChild();

                ImGui::SameLine();

                if (ImGui::BeginChild("editor")) {
                    renderEditor(registry, e);
                }
                ImGui::EndChild();
            }
            ImGui::End();
        }
    }
};

} // MM

// MIT License

// Copyright (c) 2019-2021 Erik Scholz
// Copyright (c) 2020 Gnik Droy

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
