#include "command_gui.hpp"
#include <gfx/imgui/imgui.h>

GuiResult DialogGui::draw_gui()
{
    bool show = true;
    ImGui::OpenPopup(this->name().data());
    if (ImGui::BeginPopupModal(this->name().data(), &show, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
        this->draw_dialog();

        ImGui::Separator();

        if (ImGui::Button("Run")) {
            ImGui::EndPopup();
            return GuiResult::RunAndClose;
        }

        ImGui::EndPopup();
    }

    return show ? GuiResult::KeepOpen : GuiResult::Close;
}