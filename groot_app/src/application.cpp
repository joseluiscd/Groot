#include <gfx/font_awesome.hpp>
#include <gfx/glad.h>
#include <gfx/imgui/gfx.hpp>
#include <gfx/imgui/imgui.h>
#include <gfx/render_pass.hpp>
#include <groot_app/app_log.hpp>
#include <groot_app/application.hpp>
#include <groot_app/cloud_io.hpp>
#include <groot_app/cloud_system.hpp>
#include <groot_app/command.hpp>
#include <groot_app/components.hpp>
#include <groot_app/create_graph.hpp>
#include <groot_app/cylinder_connect.hpp>
#include <groot_app/cylinder_marching.hpp>
#include <groot_app/graph_cluster.hpp>
#include <groot_app/graph_io.hpp>
#include <groot_app/graph_repair.hpp>
#include <groot_app/graph_resample.hpp>
#include <groot_app/graph_viewer_system.hpp>
#include <groot_app/render.hpp>
#include <groot_app/screenshot.hpp>
#include <groot_app/viewer_system.hpp>
#include <groot_app/workspace_io.hpp>
#include <spdlog/spdlog.h>

struct ApplicationProperties {
    glm::vec4 bg_color { 0.0, 0.1, 0.3, 0.0 };

    void draw_window(bool* open = nullptr);
};

void ApplicationProperties::draw_window(bool* open)
{
    if (ImGui::Begin("Application Properties", open)) {
        ImGui::ColorEdit4("Background color", &bg_color[0]);
    }
    ImGui::End();
}

namespace viewer_system {
void python();
}

gfx::InitOptions get_options()
{
    gfx::InitOptions options;
    options.title = "Groot Graph Viewer";
    options.maximized = true;
    options.resizable = true;
    options.imgui = true;
    options.debug_draw = true;
    options.debug_context = true;
    return options;
};

Application::Application(entt::registry& _reg)
    : gui_app(get_options())
    , registry(_reg)
    , app_log(new AppLog)
{
    registry.set<EntityEditor>();
    registry.set<SelectedEntity>();
    registry.set<TaskBroker>();
    registry.set<ApplicationProperties>();

    init_components(registry);
    ShaderCollection::init(registry);
    viewer_system::init(registry);
    graph_viewer_system::init(registry);
    cloud_view_system::init(registry);
    cylinder_view_system::init(registry);

    registry.set<ImGuiContext*>(ImGui::GetCurrentContext());

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
    try {
        sched.run_all_tasks();
    } catch (const std::exception& e) {
        spdlog::error("{}", e.what());
    }

    if (windows.background_tasks && ImGui::Begin("Background tasks", &windows.background_tasks)) {
        registry.ctx<TaskBroker>().cycle_tasks([](std::string_view&& task_name) {
            ImGui::Spinner("##spinner", 10.0f, 5.0f);
            ImGui::SameLine();
            ImGui::Text("%s", task_name.data()); }, [](const std::string_view& error) { spdlog::error("{}", error); });
    } else {
        registry.ctx<TaskBroker>().cycle_tasks([](auto&& _) {}, [](const std::string_view& error) { spdlog::error("{}", error); });
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
    ApplicationProperties& props = registry.ctx<ApplicationProperties>();

    ImGui::BeginMainWindow();
    auto get_selection = [this]() {
        return std::vector<entt::entity>({ this->get_selected_entity() });
    };

    auto get_selected_handle = [this]() {
        return entt::handle(registry, this->get_selected_entity());
    };

    viewer_system::python();

    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem(ICON_FA_FOLDER_OPEN "\tOpen Workspace")) {
                open_new_window<OpenWorkspace>();
            }

            if (ImGui::MenuItem(ICON_FA_SAVE "\tSave Workspace")) {
                open_new_window<SaveWorkspace>();
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
            ImGui::Separator();

            if (ImGui::MenuItem(ICON_FA_IMAGE "\tSave 3D window screenshot")) {
                take_screenshot(registry);
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
                open_new_window<ComputeNormals>(get_selected_handle());
            }

            if (ImGui::MenuItem(ICON_FA_CUBES "\tSplit Voxels...")) {
                open_new_window<SplitCloudGui>(get_selected_handle());
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
            if (ImGui::MenuItem(ICON_FA_FILE_IMPORT "\tImport PlantGraph")) {
                open_new_window<ImportGraphGui>();
            }

            if (ImGui::MenuItem(ICON_FA_FILE_EXPORT "\tExport PlantGraph")) {
                open_new_window<ExportGraphGui>(get_selected_handle());
            }

            ImGui::Separator();
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tResample Graph")) {
                open_new_window<GraphResampleGui>(get_selection());
            }
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tCompute connected components")) {
                registry.ctx<TaskBroker>().push_task(
                    "Computing connected components",
                    graph_compute_connected_components(get_selected_handle()));
            }
            if (ImGui::MenuItem(ICON_FA_HAMMER "\tRepair connectivity")) {
                registry.ctx<TaskBroker>().push_task(
                    "Graph repair",
                    graph_repair_command(get_selected_handle()));
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
            ImGui::MenuItem("Application properties", nullptr, &windows.application_properties);
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
        gfx::RenderPass _(fbo, gfx::ClearOperation::color_and_depth(props.bg_color));
        glEnable(GL_DEPTH_TEST);
    }

    ImGui::SetNextWindowSize(ImVec2(200, 200), ImGuiCond_FirstUseEver);
    bool will_render = false;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(1.0f, 1.0f));
    if (ImGui::Begin("3D Viewer")) {
        will_render = true;
    }
    ImGui::PopStyleVar();
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
    bool bg_tasks_empty = registry.ctx<TaskBroker>().empty();

    if (ImGui::Begin("Entity properties")) {
        entity_editor.renderEditor(registry, selected, !bg_tasks_empty);
    }
    ImGui::End();

    draw_command_gui();
    draw_background_tasks();
    draw_console_log();

    if (windows.application_properties) {
        props.draw_window(&windows.application_properties);
    }

    if (windows.demo_window)
        ImGui::ShowDemoWindow(&windows.demo_window);

    ImGui::EndMainWindow();
}

void Application::draw_console_log()
{
    if (windows.console_log) {
        app_log->draw("Log", &windows.console_log);
    }
}

bool Application::should_close()
{
    return gui_app.should_close();
}

void Application::step_gui()
{
    gui_app.step([&]() {
        draw_gui();
        gui_app.draw_gui();
    });
}

void Application::step_gui(std::function<void(entt::registry&)> update)
{
    gui_app.step([&]() {
        draw_gui();
        update(registry);
        gui_app.draw_gui();
    });
}

void Application::main_loop()
{
    gui_app.main_loop([&]() {
        draw_gui();
        gui_app.draw_gui();
    });
}

void Application::main_loop(std::function<void(entt::registry&)> update)
{
    gui_app.main_loop([&]() {
        draw_gui();
        update(registry);
        gui_app.draw_gui();
    });
}