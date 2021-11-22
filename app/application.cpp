#include "application.hpp"
#include "app_log.hpp"
#include "cloud_io.hpp"
#include "cloud_system.hpp"
#include "command.hpp"
#include "components.hpp"
#include "create_graph.hpp"
#include "cylinder_connect.hpp"
#include "cylinder_marching.hpp"
#include "entt/entity/fwd.hpp"
#include "gfx/font_awesome.hpp"
#include "graph_cluster.hpp"
#include "graph_resample.hpp"
#include "graph_viewer_system.hpp"
#include "open_workspace.hpp"
#include "render.hpp"
#include "save_workspace.hpp"
#include "viewer_system.hpp"
#include <gfx/glad.h>
#include <gfx/imgui/gfx.hpp>
#include <gfx/imgui/imgui.h>
#include <gfx/render_pass.hpp>
#include <spdlog/spdlog.h>

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
    , app_log(new AppLog)
{
    registry.set<EntityEditor>();
    registry.set<SelectedEntity>();
    registry.set<TaskBroker>();

    init_components(registry);
    ShaderCollection::init(registry);
    viewer_system::init(registry);
    graph_viewer_system::init(registry);
    cloud_view_system::init(registry);
    cylinder_view_system::init(registry);

    spdlog::default_logger()->sinks().push_back(app_log);
}

Application::~Application()
{
    ShaderCollection::deinit(registry);
    viewer_system::deinit(registry);
    graph_viewer_system::deinit(registry);
    cloud_view_system::deinit(registry);
    cylinder_view_system::deinit(registry);
}

void Application::open_window(Gui* gui)
{
    guis.emplace_back(gui);
}

void Application::show_error(const std::string& error)
{
    spdlog::error("{}", error);
}

void Application::draw_background_tasks()
{
    async::fifo_scheduler& sched = sync_scheduler();
    sched.run_all_tasks();

    if (windows.background_tasks && ImGui::Begin("Background tasks", &windows.background_tasks)) {
        registry.ctx<TaskBroker>().cycle_tasks([](std::string_view&& task_name) {
            ImGui::Spinner("##spinner", 10.0f, 5.0f);
            ImGui::SameLine();
            ImGui::Text("%s", task_name.data());
        });
    } else {
        registry.ctx<TaskBroker>().cycle_tasks([](auto&& _) {});
    }

    ImGui::End();
}

void Application::draw_command_gui()
{
    std::vector<Command*> commands;
    bool erase = false;

    auto it = guis.begin();
    while (it != guis.end()) {
        switch ((*it)->draw_gui()) {
        case GuiResult::Close:
            guis.erase(it++);
            break;

        case GuiResult::KeepOpen:
            ++it;
            break;

        case GuiResult::RunAndClose:
            erase = true;
            [[fallthrough]];
        case GuiResult::RunAndKeepOpen:
            (*it)->schedule_commands(registry);
            if (erase)
                guis.erase(it++);
            break;
        }
    }
}

entt::entity Application::get_selected_entity()
{
    return registry.ctx<SelectedEntity>().selected;
}

void Application::draw_gui()
{
    ImGui::BeginMainWindow();
    auto get_selection = [this]() {
        return std::vector<entt::entity>({ this->get_selected_entity() });
    };

    auto get_selected_handle = [this]() {
        return entt::handle(registry, this->get_selected_entity());
    };

    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem(ICON_FA_FOLDER_OPEN "\tOpen Workspace")) {
                open_new_window_adaptor<OpenWorkspace>();
            }

            if (ImGui::MenuItem(ICON_FA_SAVE "\tSave Workspace")) {
                open_new_window_adaptor<SaveWorkspace>(registry);
            }

            ImGui::Separator();

            if (ImGui::MenuItem(ICON_FA_TRASH "\tClear Invisible")) {
                registry.each([&](entt::entity e) {
                    if (!registry.all_of<Visible>(e)) {
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
                open_new_window<ImportPLYGui>();
            }

            if (ImGui::MenuItem(ICON_FA_FILE_EXPORT "\tExport PLY")) {
                open_new_window<ExportPLYGui>(get_selected_handle());
            }

            ImGui::Separator();

            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tNormals...")) {
                open_new_window_adaptor<ComputeNormals>(registry);
            }

            if (ImGui::MenuItem(ICON_FA_CUBES "\tSplit Voxels...")) {
                open_new_window_adaptor<SplitCloud>(registry);
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Plant Graph")) {
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tCreate Graph from cloud...")) {
                open_new_window_adaptor<CreateGraph>(registry);
            }
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tClustering...")) {
                open_new_window_adaptor<GraphCluster>(registry);
            }
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tCompute from cylinders")) {
                open_new_window_adaptor<CylinderConnection>(registry);
            }
            ImGui::Separator();
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tGeodesic graph")) {
                open_new_window_adaptor<GeodesicGraphCommand>(registry);
            }
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tMinimum Spanning Tree")) {
                open_new_window_adaptor<MSTGraphCommand>(registry);
            }
            ImGui::Separator();
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tResample Graph")) {
                open_new_window<GraphResampleGui>(get_selection());
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Cylinders")) {
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tCylinders...")) {
                open_new_window_adaptor<CylinderMarching>(registry);
            }
            if (ImGui::MenuItem(ICON_FA_FILTER "\tFilter Cylinders...")) {
                open_new_window_adaptor<CylinderFilter>(registry);
            }
            if (ImGui::MenuItem(ICON_FA_CUBE "\tBuild cloud from cylinders")) {
                open_new_window_adaptor<CylinderPointFilter>(registry);
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Window")) {
            ImGui::MenuItem("Background tasks", nullptr, &windows.background_tasks);
            ImGui::MenuItem("Demo window", nullptr, &windows.demo_window);
            ImGui::MenuItem("Logger", nullptr, &windows.console_log);

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
        gfx::RenderPass _(fbo, gfx::ClearOperation::color_and_depth({ 0.0, 0.1, 0.3, 0.0 }));
        glEnable(GL_DEPTH_TEST);
    }

    bool will_render = false;
    if (ImGui::Begin("3D Viewer")) {
        will_render = true;
    }
    ImGui::End();

    if (will_render) {
        graph_viewer_system::run(registry);
        cloud_view_system::run(registry);
        cylinder_view_system::run(registry);
        viewer_system::run(registry);
    }

    auto& entity_editor = registry.ctx<EntityEditor>();
    if (ImGui::Begin("Entity List")) {
        entity_editor.renderEntityList(registry, entity_filter);
    }
    ImGui::End();

    auto selected = registry.ctx<SelectedEntity>().selected;

    if (ImGui::Begin("Entity properties")) {
        bool bg_tasks_empty = registry.ctx<TaskBroker>().empty();
        entity_editor.renderEditor(registry, selected, !bg_tasks_empty);
    }
    ImGui::End();

    draw_command_gui();
    draw_background_tasks();
    draw_console_log();

    if (windows.demo_window)
        ImGui::ShowDemoWindow(&windows.demo_window);

    ImGui::EndMainWindow();
    gui_app.draw_gui();
}

void Application::draw_console_log()
{
    if (windows.console_log) {
        app_log->draw("Log", &windows.console_log);
    }
}

void Application::main_loop()
{
    gui_app.main_loop([&]() {
        draw_gui();
    });
}
