// for the license, see the end of the file
#pragma once

#include <functional>
#include <map>
#include <set>
#include <string>

#include <gfx/imgui/imgui.h>
#include <groot/assert.hpp>
#include <groot_app/components.hpp>
#include <groot_app/entt.hpp>

namespace MM {

using EntityType = entt::entity;
using RegistryType = entt::registry;

inline void EntityWidget(EntityType& e, RegistryType& reg, bool dropTarget = false)
{
    ImGui::PushID(static_cast<int>(entt::to_integral(e)));

    if (reg.valid(e)) {
        const char* name;
        char namebuff[32];
        auto& selected = reg.template ctx<SelectedEntity>().selected;
        bool visible = reg.template all_of<Visible>(e);

        if (reg.template all_of<Name>(e)) {
            name = reg.template get<Name>(e).name.c_str();
        } else {
            snprintf(namebuff, 32, "Entity %x\0", entt::to_integral(e));
            name = namebuff;
        }

        ImGui::PushID("###Checkbox");
        ImGui::PushID((void*)e);
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
        if (ImGui::Selectable(name, selected == e)) {
            selected = e;
        }

    } else {
        ImGui::Text("Invalid Entity");
    }

    ImGui::PopID();
}

template <class Component>
void ComponentEditorWidget([[maybe_unused]] RegistryType& registry, [[maybe_unused]] EntityType entity) { }

template <class Component>
void ComponentAddAction(RegistryType& registry, EntityType entity)
{
    registry.template emplace<Component>(entity);
}

template <class Component>
void ComponentRemoveAction(RegistryType& registry, EntityType entity)
{
    registry.template remove<Component>(entity);
}

class EntityEditor {
public:
    using Registry = RegistryType;
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
        GROOT_ASSERT(insert_info.second, "Failed to insert");
        return std::get<ComponentInfo>(*insert_info.first);
    }

    template <class Component>
    ComponentInfo& registerComponent(const std::string& name, typename ComponentInfo::Callback widget, bool alt_color = false)
    {
        return registerComponent<Component>(ComponentInfo {
            name,
            widget,
            ComponentAddAction<Component>,
            ComponentRemoveAction<Component>,
            alt_color });
    }

    template <class Component>
    ComponentInfo& registerComponent(const std::string& name, bool alt_color = false)
    {
        return registerComponent<Component>(name, ComponentEditorWidget<Component>, alt_color);
    }

    void renderEditor(Registry& registry, EntityType& e, bool disable_delete = false);

    void renderEntityList(Registry& registry, std::set<ComponentTypeID>& comp_list);
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
