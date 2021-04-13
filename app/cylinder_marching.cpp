#include "cylinder_marching.hpp"
#include "components.hpp"
#include "resources.hpp"
#include <future>
#include <gfx/imgui/imgui.h>
#include <queue>

CylinderMarching::CylinderMarching(entt::registry& _reg)
    : reg(_reg)
{
    target = reg.ctx<SelectedEntity>().selected;

    if (reg.valid(target) && reg.all_of<PointCloud, PointNormals>(target)) {
        std::tie(*cloud, *normals) = reg.get<PointCloud, PointNormals>(target);

    } else {
        throw std::runtime_error("Selected entity must have PointCloud and PointNormals");
    }
}

GuiState CylinderMarching::draw_gui()
{
    bool show = true;
    ImGui::OpenPopup("Cylinder marching");
    if (ImGui::BeginPopupModal("Cylinder marching", &show, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {

        ImGui::Separator();

        ImGui::InputFloat("Epsilon", &params.epsilon, 0.05, 0.1);
        ImGui::InputFloat("Normal Threshold", &params.normal_threshold, 0.1, 0.5);
        ImGui::InputFloat("Cluster epsilon", &params.cluster_epsilon);
        ImGui::InputInt("Min points", (int*)&params.min_points);
        ImGui::InputFloat("Missing probability", &params.probability);

        ImGui::Separator();

        ImGui::Separator();

        if (ImGui::Button("Run")) {
            ImGui::EndPopup();
            return GuiState::RunAsync;
        }

        ImGui::EndPopup();
    }

    return show ? GuiState::Editing : GuiState::Close;
}

CommandState CylinderMarching::execute()
{
    groot::compute_cylinders(cloud->cloud.data(), normals->normals.data(), cloud->cloud.size(), result, params);
    return CommandState::Ok;
}

void CylinderMarching::on_finish()
{
    reg.emplace_or_replace<Cylinders>(target, std::move(result));
}
