#pragma once

#include <gfx/imgui/gfx.hpp>
#include <glm/fwd.hpp>
#include <gfx/imgui/imfilebrowser.h>
#include <groot/particles.hpp>

class ParticleSim {
public:
    ParticleSim();
    void draw_gui();
    void step_simulation();

    void draw_gui_config();
    void draw_gui_view();

    enum State {
        Configuration,
        Visualization
    };

private:
    State state;
    groot::Particles particles;
    ImGui::FileBrowser file_browser;

};
