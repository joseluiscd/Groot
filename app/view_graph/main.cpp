#include "imfilebrowser.h"
#include <fstream>
#include <gfx/camera.hpp>
#include <gfx/debug_draw.hpp>
#include <gfx/gfx.hpp>
#include <gfx/glad.h>
#include <gfx/imgui/imgui.h>
#include <groot/skeleton.hpp>
#include <groot/toml.hpp>
#include <lemon/list_graph.h>
#include <spdlog/spdlog.h>
#include <GLFW/glfw3.h>
#include "create_graph.hpp"
#include "graph_view.hpp"


using Cloud = pcl::PointCloud<pcl::PointXYZ>;

void draw_graph(const groot::PlantGraph& graph)
{
    //dd::sphere(glm::vec3(0, 0, 0), dd::colors::Red, 1.0);
    for (groot::NodeIt i(*graph.graph); i != lemon::INVALID; ++i) {
        glm::vec3 point = (*graph.point)[i];
        dd::point(point, dd::colors::WhiteSmoke, 5.0f);
    }

    for (groot::EdgeIt i(*graph.graph); i != lemon::INVALID; ++i) {
        glm::vec3 origin = (*graph.point)[graph.graph->u(i)];
        glm::vec3 dest = (*graph.point)[graph.graph->v(i)];

        dd::line(origin, dest, dd::colors::Red, 2.0);
    }
}

int main()
{
    gfx::Gfx app(gfx::InitOptions {
        .title = "Groot Graph Viewer",
        .maximized = true,
        .resizable = true,
        .debug_draw = true,
        .debug_context = true,
    });

    ImGui::FileBrowser file_dialog;
    file_dialog.SetTitle("Graph Open");
    file_dialog.SetTypeFilters({ ".lgf" });

    gfx::PerspectiveCameraLens lens(70.0, 1.0, 0.2, 100.2);
    app.get_surface().add_resize_observer(lens);

    gfx::CameraRig cam = gfx::CameraRig(lens)
                             .with_position({ 0.0, 0.0, 20.0 })
                             .look_at({ 0.0, 0.0, 0.0 })
                             .with_up_vector({ 0.0, 1.0, 0.0 });

    gfx::DebugRender dr = app.debug_render(app.get_surface(), cam);

    groot::PlantGraph graph = groot::PlantGraph::empty();

    bool grabbed_primary = false;
    bool grabbed_secondary = false;
    glm::vec2 old_mouse_pos;

    app.set_mouse_button_callback([&](int button, int action, int mods) {
        old_mouse_pos = app.get_mouse_position();
        if (action == GLFW_PRESS) {
            switch (button)
            {
            case GLFW_MOUSE_BUTTON_1:
                grabbed_primary = true;
                break;
            
            case GLFW_MOUSE_BUTTON_2:
                grabbed_secondary = true;
                break;
            }
        } else if (action == GLFW_RELEASE) {
            switch (button)
            {
            case GLFW_MOUSE_BUTTON_1:
                grabbed_primary = false;
                break;
            
            case GLFW_MOUSE_BUTTON_2:
                grabbed_secondary = false;
                break;
            }
        }
    });

    app.set_mouse_move_callback([&](glm::vec2 position) {
        if (grabbed_secondary) {
            glm::vec2 diff = (position - old_mouse_pos) * 0.01f;
            old_mouse_pos = position;
            cam.truck(-diff.x);
            cam.crane(diff.y);
        } else if (grabbed_primary) {
            glm::vec2 diff = (position - old_mouse_pos) * 0.01f;
            old_mouse_pos = position;
            cam.orbit(diff.x);
            cam.orbit_vertical(diff.y);
        }
    });

    GraphViewer viewer(graph);

    auto update_graph = [&](groot::PlantGraph&& g) {
        graph = std::move(g);
        viewer.set_plant_graph(graph);
        spdlog::info("Setting plant graph");
    };


    CreateGraph graph_creator(update_graph);

    bool style_editor = false;

    app.main_loop([&]() {
        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Open PLY")) {
                    graph_creator.show();
                }

                if (ImGui::MenuItem("Open Graph", "CTRL+O")) {
                    file_dialog.Open();
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Edit")) {
                if (ImGui::MenuItem("Styles")) {
                    style_editor = true;
                }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        if(style_editor) {
            ImGui::ShowStyleEditor();
        }
        
        file_dialog.Display();
        if (file_dialog.HasSelected()) {
            std::ifstream file(file_dialog.GetSelected());

            graph = groot::PlantGraph::read_from_file(file);
            viewer.set_plant_graph(graph);
            
            //glm::vec3 root = (*graph.point)[graph.root];
            //cam.look_at(root);

            file_dialog.ClearSelected();
        }

        graph_creator.run_gui();


        //draw_graph(graph);
        //dd::sphere(glm::vec3(0, 0, 0), dd::colors::Red, 10.0);
        viewer.render(app.get_surface(), cam);

        dr.draw();
    });
}