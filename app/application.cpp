#include "application.hpp"
#include <gfx/imgui/gfx.hpp>
#include <gfx/imgui/imgui.h>
#include <gfx/render_pass.hpp>
#include <spdlog/spdlog.h>
#include <gfx/glad.h>
#include "render.hpp"
#include <gfx/imgui/gfx.hpp>
#include <gfx/font_awesome.hpp>
#include <boost/stacktrace.hpp>
#include <type_traits>
#include "cloud_system.hpp"
#include "cloud_io_system.hpp"
#include "viewer_system.hpp"

void ui_wait_handler(async::task_wait_handle h)
{
    std::stringstream ss;
    ss << boost::stacktrace::stacktrace();

    spdlog::error("Waiting for async task on the main thread. \n{}", ss.str());
}

Application::Application(entt::registry& _reg)
    : bait::Application(_reg, gfx::InitOptions {
        .title = "Groot Graph Viewer",
        .maximized = true,
        .resizable = true,
        .imgui = true,
        .debug_draw = true,
        .debug_context = true,
    })
    , systems(bait::make_dynamic_system_collection<
        CloudSystem,
        CloudIOSystem,
        ViewerSystem>())
    , global(registry.create())
{
    registry.set<EntityEditor>();

    init_components(registry);
    ShaderCollection::init(registry);

    systems->init(registry);

    async::set_thread_wait_handler(ui_wait_handler);
}

Application::~Application()
{
    ShaderCollection::deinit(registry);
    systems->clear(registry);
}

void Application::draw_gui()
{
    ImGui::BeginMainWindow();

    if (ImGui::BeginMenuBar()) {
        /*
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
        */
        if (ImGui::BeginMenu("Point Cloud")) {
            if (ImGui::MenuItem(ICON_FA_FILE_IMPORT "\tImport PLY")) {
                registry.emplace<bait::GuiTarget<ImportPLY>>(global);
            }

            /*
            if (ImGui::MenuItem(ICON_FA_FILE_EXPORT "\tExport PLY")) {
                open_new_window<ExportPLY>(registry);
            }

            ImGui::Separator();
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tNormals...")) {
                open_new_window<ComputeNormals>(registry);
            }

            if (ImGui::MenuItem(ICON_FA_CUBES "\tSplit Voxels...")) {
                open_new_window<SplitCloud>(registry);
            }*/

            ImGui::EndMenu();
        }
        /*
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
        */
        ImGui::EndMenuBar();
    }
    {
        auto& fbo = registry.ctx<RenderData>().framebuffer;
        gfx::RenderPass(fbo, gfx::ClearOperation::color_and_depth({ 1.0, 1.0, 1.0, 0.0 }));
        //gfx::RenderPass(fbo, gfx::ClearOperation::color_and_depth({ 0.0, 0.1, 0.3, 0.0 }));
        glEnable(GL_DEPTH_TEST);
    }


    auto& entity_editor = registry.ctx<EntityEditor>();
    if (ImGui::Begin("Entity List")) {
        entity_editor.renderEntityList(registry, entity_filter);
    }
    ImGui::End();

    /*auto selected = registry.ctx<SelectedEntity>().selected;

    if (ImGui::Begin("Entity properties")) {
        entity_editor.renderEditor(registry, selected);
    }
    ImGui::End();
    */

    if (windows.demo_window)
        ImGui::ShowDemoWindow(&windows.demo_window);

    ImGui::EndMainWindow();
    gui_app.draw_gui();
}

void Application::main_loop()
{
    gui_app.main_loop([&]() {
        systems->update(registry);
        draw_gui();
    });
}
