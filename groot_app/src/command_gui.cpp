#include <gfx/imgui/imgui.h>
#include <groot_app/command_gui.hpp>

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

const int open_flags = ImGuiFileBrowserFlags_CloseOnEsc;
const int save_flags = open_flags | ImGuiFileBrowserFlags_CreateNewDir | ImGuiFileBrowserFlags_EnterNewFilename;

FileDialogGui::FileDialogGui(
    FileDialogType type,
    const std::string& name,
    const std::vector<std::string>& typeFilters)
{
    switch (type) {
    case FileDialogType::Open:
        file_dialog = ImGui::FileBrowser(open_flags);
        break;
    case FileDialogType::Save:
        file_dialog = ImGui::FileBrowser(save_flags);
        break;
    }

    file_dialog.SetTitle(name);
    file_dialog.SetTypeFilters(typeFilters);
    file_dialog.Open();
}

GuiResult FileDialogGui::draw_gui()
{
    file_dialog.Display();

    if (file_dialog.HasSelected()) {
        selected_file = file_dialog.GetSelected();
        file_dialog.ClearSelected();
        return GuiResult::RunAndClose;
    } else if (!file_dialog.IsOpened()) {
        return GuiResult::Close;
    } else {
        return GuiResult::KeepOpen;
    }
}

void GuiAdapter::schedule_commands(entt::registry& reg)
{
    std::string name(gui->name());

    reg.ctx<TaskManager>().push_task(
        name,
        create_task()
            .then_async([gui = std::exchange(gui, nullptr)]() {
                std::unique_ptr<CommandGui> cmd(gui);
                if (cmd->execute() == CommandState::Error) {
                    throw std::runtime_error(cmd->error_string);
                };
                return cmd;
            })
            .then_sync([&reg](std::unique_ptr<CommandGui>&& cmd) {
                cmd->on_finish(reg);
            })
            .build());
}

GuiResult GuiAdapter::draw_gui()
{
    switch (gui->draw_gui()) {
    case GuiState::Close:
        return GuiResult::Close;
    case GuiState::Editing:
        return GuiResult::KeepOpen;
    case GuiState::RunAsync:
        return GuiResult::RunAndClose;
    case GuiState::RunAsyncUpdate:
        return GuiResult::RunAndKeepOpen;
    default:
        return GuiResult::KeepOpen;
    }
}