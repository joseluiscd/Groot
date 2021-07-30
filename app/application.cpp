#include "application.hpp"
#include "cloud_system.hpp"
#include "components.hpp"
#include "create_graph.hpp"
#include "graph_cluster.hpp"
#include "cylinder_marching.hpp"
#include "cylinder_connect.hpp"
#include "entt/entity/fwd.hpp"
#include "gfx/font_awesome.hpp"
#include "graph_viewer_system.hpp"
#include "import_ply.hpp"
#include "open_workspace.hpp"
#include "render.hpp"
#include "save_workspace.hpp"
#include "viewer_system.hpp"
#include <gfx/imgui/gfx.hpp>
#include <gfx/imgui/imgui.h>
#include <gfx/render_pass.hpp>
#include <spdlog/spdlog.h>
#include <gfx/glad.h>

Application::Application(entt::registry& _reg)
    : gui_app(gfx::InitOptions {
        .title = "Groot Graph Viewer",
        .maximized = true,
        .resizable = true,
        .imgui = true,
        .debug_draw = true,
        .debug_context = true,
    })
    , registry(_reg)
{
    registry.set<EntityEditor>();
    registry.set<SelectedEntity>();

    init_components(registry);
    ShaderCollection::init(registry);
    viewer_system::init(registry);
    graph_viewer_system::init(registry);
    cloud_view_system::init(registry);
    cylinder_view_system::init(registry);
}

Application::~Application()
{
    ShaderCollection::deinit(registry);
    viewer_system::deinit(registry);
    graph_viewer_system::deinit(registry);
    cloud_view_system::deinit(registry);
    cylinder_view_system::deinit(registry);
}

BackgroundTaskHandle Application::execute_command_async(std::unique_ptr<Command>&& command)
{
    BackgroundTaskHandle it;

    {
        std::unique_lock _lock(background_task_lock);
        it = background_tasks.emplace(background_tasks.end(), std::make_shared<BackgroundTask>());
    }

    (*it)->command = std::move(command);
    (*it)->task = std::async([=]() {
        std::shared_lock _lock((*it)->lock);

        CommandState result = (*it)->command->execute();
        this->notify_task_finished(it, result);
    });

    return it;
}

BackgroundTaskHandle Application::execute_command_async(Command* command)
{
    return execute_command_async(std::unique_ptr<Command>(command));
}

void Application::execute_command(Command* command)
{
    switch (command->execute()) {
    case CommandState::Error:
        this->show_error(command->error_string);
        break;

    default:
        break;
    }
}

void Application::notify_task_finished(BackgroundTaskHandle task, CommandState result)
{
    std::unique_lock _lock(background_task_lock);
    if (result == CommandState::Error) {
        this->show_error((*task)->command->error_string);
    }
    remove_background_tasks.push(*task);
    background_tasks.erase(task);
}

void Application::open_window(CommandGui* gui)
{
    std::unique_lock _lock(command_gui_lock);

    command_guis.emplace_back(gui);
}

void Application::show_error(const std::string& error)
{
    spdlog::error("{}", error);
}


void Application::draw_background_tasks()
{
    if (ImGui::Begin("Background tasks", &windows.background_tasks)) {
        std::shared_lock _lock(background_task_lock);
        for (auto it = background_tasks.begin(); it != background_tasks.end(); ++it) {
            ImGui::Spinner("##spinner", 10.0f);
            ImGui::SameLine();
            ImGui::Text("Background task 0x%zx", (size_t)it->get());
        }
    }
    ImGui::End();
}

void Application::draw_command_gui()
{
    std::unique_lock _lock(command_gui_lock);

    auto it = command_guis.begin();
    while (it != command_guis.end()) {
        switch ((*it)->draw_gui()) {
        case GuiState::Close:
            command_guis.erase(it++);
            break;

        case GuiState::Editing:
            ++it;
            break;

        case GuiState::RunSync:
            this->execute_command(it->get());
            command_guis.erase(it++);
            break;
        case GuiState::RunAsync:
            this->execute_command_async(std::move(*it));
            command_guis.erase(it++);
            break;
        case GuiState::RunAsyncUpdate:
            this->execute_command_async(std::move(*it));
            ++it;
            break;
        }
    }
}

void Application::draw_gui()
{
    ImGui::BeginMainWindow();

    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem(ICON_FA_FOLDER_OPEN "\tOpen Workspace")) {
                open_new_window<OpenWorkspace>(registry);
            }

            if (ImGui::MenuItem(ICON_FA_SAVE "\tSave Workspace")) {
                open_new_window<SaveWorkspace>(registry);
            }

            ImGui::Separator();

            if (ImGui::MenuItem(ICON_FA_TRASH "\tClear Invisible")) {
                registry.each([&](entt::entity e){
                    if (! registry.all_of<Visible>(e)) {
                        registry.destroy(e);
                    }
                });
            }

            if (ImGui::MenuItem(ICON_FA_TRASH "\tClear Workspace")) {
                registry.clear();
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            if (ImGui::MenuItem(ICON_FA_EYE_SLASH "\tHide all")) {
                registry.clear<Visible>();
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Point Cloud")) {
            if (ImGui::MenuItem(ICON_FA_FILE_IMPORT "\tImport PLY")) {
                open_new_window<ImportPLY>(registry);
            }

            if (ImGui::MenuItem(ICON_FA_FILE_EXPORT "\tExport PLY")) {
                open_new_window<ExportPLY>(registry);
            }

            ImGui::Separator();

            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tNormals...")) {
                open_new_window<ComputeNormals>(registry);
            }

            if (ImGui::MenuItem(ICON_FA_CUBES "\tSplit Voxels...")) {
                open_new_window<SplitCloud>(registry);
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Plant Graph")) {
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tCreate Graph from cloud...")) {
                open_new_window<CreateGraph>(registry);
            }
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tClustering...")) {
                open_new_window<GraphCluster>(registry);
            }
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tCompute from cylinders")) {
                open_new_window<CylinderConnection>(registry);
            }
            ImGui::Separator();
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tGeodesic graph")) {
                open_new_window<GeodesicGraphCommand>(registry);
            }
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tMinimum Spanning Tree")) {
                open_new_window<MSTGraphCommand>(registry);
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Cylinders")) {
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tCylinders...")) {
                open_new_window<CylinderMarching>(registry);
            }
            if (ImGui::MenuItem(ICON_FA_FILTER "\tFilter Cylinders...")) {
                open_new_window<CylinderFilter>(registry);
            }
            if (ImGui::MenuItem(ICON_FA_CUBE "\tBuild cloud from cylinders")) {
                open_new_window<CylinderPointFilter>(registry);
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Help")) {
            if (ImGui::MenuItem(ICON_FA_BUG "\tDemo window")) {
                windows.demo_window = true;
            }
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }

    {
        auto& fbo = registry.ctx<RenderData>().framebuffer;
        gfx::RenderPass(fbo, gfx::ClearOperation::color_and_depth({ 0.0, 0.1, 0.3, 0.0 }));
        glEnable(GL_DEPTH_TEST);
    }

    graph_viewer_system::run(registry);
    cloud_view_system::run(registry);
    cylinder_view_system::run(registry);
    viewer_system::run(registry);

    auto& entity_editor = registry.ctx<EntityEditor>();
    if (ImGui::Begin("Entity List")) {
        entity_editor.renderEntityList(registry, entity_filter);
    }
    ImGui::End();

    auto selected = registry.ctx<SelectedEntity>().selected;

    if (ImGui::Begin("Entity properties")) {
        entity_editor.renderEditor(registry, selected);
    }
    ImGui::End();

    draw_command_gui();
    draw_background_tasks();

    if (windows.demo_window)
        ImGui::ShowDemoWindow(&windows.demo_window);

    ImGui::EndMainWindow();
    gui_app.draw_gui();
}

void Application::main_loop()
{
    gui_app.main_loop([&]() {
        while (!remove_background_tasks.empty()) {
            remove_background_tasks.front()->command->on_finish();
            remove_background_tasks.pop();
        }

        draw_gui();
    });
}
