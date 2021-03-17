#pragma once

#include <gfx/imgui/gfx.hpp>
#include <glm/fwd.hpp>
#include <gfx/imgui/imfilebrowser.h>
#include <groot/particles.hpp>
#include <groot/cylinder_marching.hpp>
#include <filesystem>

class ParticleSim {
public:
    ParticleSim();
    void draw_gui();

    void draw_gui_config();
    void draw_gui_view();

    void run_gui_config();

private:
    bool config = true;
    bool visualize = false;

    //groot::Particles particles;
    ImGui::FileBrowser file_browser;
    std::filesystem::path selected_file;

    std::vector<groot::cgal::Point_3> cloud;
    std::vector<groot::Curvature> curvatures;

    int k = 5;


};
