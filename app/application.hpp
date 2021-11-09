#pragma once

#include <bait/bait.hpp>
#include <bait/system.hpp>
#include "entity_editor.hpp"
#include <set>


struct Windows {
    bool background_tasks = true;
    bool history = true;
    bool main_viewer = true;
    bool console_log = true;
    bool demo_window = false;
};

class Application : public bait::Application {
public:
    using EntityEditor = MM::EntityEditor<entt::entity>;

    Application(entt::registry& reg);

    ~Application();

    void draw_gui();
    void draw_command_gui();
    void draw_background_tasks();

    void main_loop();

private:
    entt::entity get_selected_entity();

    std::set<EntityEditor::ComponentTypeID> entity_filter;

    bait::AbstractSystem::Ptr systems;

    Windows windows;
};
