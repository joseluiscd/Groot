#include <groot_app/entity_editor.hpp>

namespace MM {

void EntityEditor::renderEditor(Registry& registry, EntityType& e, bool disable_delete)
{
    ImGui::TextUnformatted("Editing:");
    ImGui::SameLine();

    EntityWidget(e, registry, true);

    if (ImGui::Button("New")) {
        e = registry.create();
    }
    if (registry.valid(e)) {
        ImGui::SameLine();

        // clone would go here
        // if (ImGui::Button("Clone")) {
        // auto old_e = e;
        // e = registry.create();
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
                } else if (!disable_delete) {
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

void EntityEditor::renderEntityList(Registry& registry, std::set<ComponentTypeID>& comp_list)
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
            EntityWidget(e, registry, false);
        });
    } else {
        auto view = registry.runtime_view(comp_list.begin(), comp_list.end());
        ImGui::Text("%lu Entities Matching:", view.size_hint());

        if (ImGui::BeginChild("entity list")) {
            for (auto e : view) {
                EntityWidget(e, registry, false);
            }
        }
        ImGui::EndChild();
    }
}
}