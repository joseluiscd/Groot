#include "graph_io.hpp"
#include "components.hpp"
#include <gfx/imgui/imfilebrowser.h>
#include <groot/plant_graph_io.hpp>
#include <spdlog/spdlog.h>

const int open_flags = ImGuiFileBrowserFlags_CloseOnEsc;
const int save_flags = open_flags | ImGuiFileBrowserFlags_CreateNewDir | ImGuiFileBrowserFlags_EnterNewFilename;

ImportGraphGui::ImportGraphGui()
    : open(open_flags)
{
    open.SetTitle("Open OBJ Graph");
    open.SetTypeFilters({ ".obj" });
    open.Open();
}

void ImportGraphGui::schedule_commands(entt::registry& reg)
{
    reg.ctx<TaskBroker>()
        .push_task(
            "Loading Plant Graph",
            import_graph_command(reg, this->input_file));
}

GuiResult ImportGraphGui::draw_gui()
{
    open.Display();

    if (open.HasSelected()) {
        input_file = open.GetSelected().string();
        open.ClearSelected();
        return GuiResult::RunAndClose;
    } else if (!open.IsOpened()) {
        return GuiResult::Close;
    } else {
        return GuiResult::KeepOpen;
    }
}

async::task<entt::entity> import_graph_command(entt::registry& reg, const std::string_view& file)
{
    return async::spawn(async_scheduler(), [input_file = std::string(file)]() {
        spdlog::info("Loading OBJ file...");

        if (input_file.empty()) {
            throw std::runtime_error("Cannot open file");
        }

        groot::PlantGraph&& data = groot::load_plant_graph(input_file.c_str());
        return data;
    }).then(sync_scheduler(), [&reg, input_file = std::string(file)](groot::PlantGraph&& data) {
        entt::entity entity = reg.create();
        reg.emplace<groot::PlantGraph>(entity, std::move(data));
        reg.emplace<Name>(entity, input_file);

        return entity;
    });
}

ExportGraphGui::ExportGraphGui(entt::handle handle)
    : save(save_flags)
    , target(handle)
{
    save.SetTitle("Open OBJ Graph");
    save.SetTypeFilters({ ".obj" });
    save.Open();

    if (!target.valid() || !target.all_of<groot::PlantGraph>()) {
        throw std::runtime_error("Selected entity must have PlantGraph");
    }
}

GuiResult ExportGraphGui::draw_gui()
{
    save.Display();

    if (save.HasSelected()) {
        output_file = save.GetSelected().string();
        save.ClearSelected();
        return GuiResult::RunAndClose;
    } else if (!save.IsOpened()) {
        return GuiResult::Close;
    } else {
        return GuiResult::KeepOpen;
    }
}
void ExportGraphGui::schedule_commands(entt::registry& reg)
{
    reg.ctx<TaskBroker>()
        .push_task(
            "Saving Plant Graph",
            export_graph_command(target, this->output_file));
}
async::task<void> export_graph_command(entt::handle e, const std::string_view& file)
{
    return async::spawn(sync_scheduler(), [e]() {
        if (!e.valid() || !e.all_of<groot::PlantGraph>()) {
            throw std::runtime_error("Selected entity must have PlantGraph");
        }

        const groot::PlantGraph* plant = &e.get<groot::PlantGraph>();

        spdlog::info("Saving OBJ...");

        return plant;
    }).then(async_scheduler(), [file = std::string(file)](const groot::PlantGraph* plant) {

        groot::save_plant_graph(file.c_str(), *plant);
        spdlog::info("Finished saving OBJ!");
    });
}
