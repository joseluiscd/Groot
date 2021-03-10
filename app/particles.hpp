#pragma once

#include <gfx/imgui/gfx.hpp>
#include <glm/fwd.hpp>
#include <gfx/imgui/imfilebrowser.h>

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

    ImGui::FileBrowser file_browser;
    
};
