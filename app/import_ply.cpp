#include "import_ply.hpp"
#include "components.hpp"
#include <fstream>
#include <gfx/imgui/imfilebrowser.h>
#include <groot/cloud_load.hpp>
#include <spdlog/spdlog.h>

const int open_flags = ImGuiFileBrowserFlags_CloseOnEsc;
const int save_flags = open_flags | ImGuiFileBrowserFlags_CreateNewDir | ImGuiFileBrowserFlags_EnterNewFilename;

ImportPLY::ImportPLY(entt::registry& _registry)
    : registry(_registry)
    , open(open_flags)
{
    open.SetTitle("Open PLY Cloud");
    open.SetTypeFilters({ ".ply" });
    open.Open();
}

GuiState ImportPLY::draw_gui()
{
    open.Display();

    if (open.HasSelected()) {
        input_file = open.GetSelected().string();
        open.ClearSelected();
        return GuiState::RunAsync;
    } else if (! open.IsOpened()) {
        return GuiState::Close;
    } else {
        return GuiState::Editing;
    }
}

CommandState ImportPLY::execute()
{
    spdlog::info("Loading PLY file...");

    if (input_file.empty()) {
        error_string = "Cannot open file";
        return CommandState::Error;
    }

    cloud = std::move(groot::load_PLY(input_file.c_str()));

    spdlog::info("Loaded PLY file with {} points!", cloud.size());
    return CommandState::Ok;
}

void ImportPLY::on_finish()
{
    auto entity = registry.create();
    registry.emplace<PointCloud>(entity, std::move(cloud));
    registry.emplace<Name>(entity, this->input_file);
    result = entity;
}
