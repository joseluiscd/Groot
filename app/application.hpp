#pragma once

#include <bait/bait.hpp>
#include <bait/system.hpp>
#include <bait/gui_system.hpp>
#include "entity_editor.hpp"
#include <set>


struct Windows {
    bool background_tasks = true;
    bool history = true;
    bool main_viewer = true;
    bool console_log = true;
    bool demo_window = false;
    bool about = false;
};

class Application : public bait::Application {
public:
    using EntityEditor = MM::EntityEditor<entt::entity>;

    Application(entt::registry& reg);

    ~Application();

    void draw_gui();

    void step();

    template <typename Component>
    void open_gui_selection()
    {
        bait::do_with_selection(registry, [this](entt::entity e) {
            registry.emplace<bait::GuiTarget<Component>>(e);
        });
    }

private:
    entt::entity get_selected_entity();

    std::set<EntityEditor::ComponentTypeID> entity_filter;

    bait::AbstractSystem::Ptr systems;

    entt::entity global;
    Windows windows;
};
