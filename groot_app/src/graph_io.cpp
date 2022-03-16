#include <groot_app/components.hpp>
#include <groot_app/graph_io.hpp>
#include <groot_graph/plant_graph_io.hpp>
#include <spdlog/spdlog.h>

ImportGraphGui::ImportGraphGui()
    : FileDialogGui(FileDialogType::Open, "Open OBJ Graph", { ".obj" })
{
}

void ImportGraphGui::schedule_commands(entt::registry& reg, const std::string& filename)
{
    reg.ctx<TaskManager>()
        .push_task(
            "Loading Plant Graph",
            import_graph_command(reg, filename));
}

async::task<entt::entity> import_graph_command(entt::registry& reg, const std::string_view& file)
{
    return create_task()
        .then_async([input_file = std::string(file)]() {
            spdlog::info("Loading OBJ file...");

            if (input_file.empty()) {
                throw std::runtime_error("Cannot open file");
            }

            groot::PlantGraph&& data = groot::load_plant_graph(input_file.c_str());
            return data;
        })
        .then_sync([&reg, input_file = std::string(file)](groot::PlantGraph&& data) {
            entt::entity entity = reg.create();
            reg.emplace<groot::PlantGraph>(entity, std::move(data));
            reg.emplace<Name>(entity, input_file);

            return entity;
        });
}

ExportGraphGui::ExportGraphGui(entt::handle handle)
    : FileDialogGui(FileDialogType::Save, "Save OBJ Graph", { ".obj" })
    , target(handle)
{
    handle_require_components<groot::PlantGraph>(target);
}

void ExportGraphGui::schedule_commands(entt::registry& reg, const std::string& filename)
{
    reg.ctx<TaskManager>()
        .push_task(
            "Saving Plant Graph",
            export_graph_command(target, filename));
}

async::task<void> export_graph_command(entt::handle e, const std::string_view& file)
{
    return create_task()
        .require_component<groot::PlantGraph>(e)
        .then_async([file = std::string(file)](const groot::PlantGraph* plant) {
            spdlog::info("Saving OBJ...");
            groot::save_plant_graph(file.c_str(), *plant);
            spdlog::info("Finished saving OBJ!");
        });
}
