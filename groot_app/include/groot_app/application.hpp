#pragma once

#include <groot_app/groot_app.hpp>
#include <groot_app/command.hpp>
#include <groot_app/command_gui.hpp>
#include <groot_app/resources.hpp>
#include <gfx/gfx.hpp>
#include <list>
#include <async++.h>
#include <groot_app/app_log.hpp>
#include <groot_app/task.hpp>

class Application;

struct GROOT_APP_LOCAL Windows {
    bool background_tasks = true;
    bool history = true;
    bool main_viewer = true;
    bool console_log = true;
    bool demo_window = false;
    bool application_properties = false;
};

class GROOT_APP_API Application {
public:
    Application(entt::registry& reg);
    ~Application();

    Application(const Application&) = delete;
    Application(Application&&) = delete;

    Application& operator=(const Application&) = delete;
    Application& operator=(Application&&) = delete;

    void open_window(Gui* gui);

    template <typename T, typename... Args>
    void open_new_window(Args&&... args)
    {
        try {
            open_window(new T(std::forward<Args...>(args)...));
        } catch (std::runtime_error& err) {
            show_error(std::string(err.what()));
        }
    }

    template <typename T, typename... Args>
    void open_new_window_adaptor(Args&&... args)
    {
        try {
            open_window(make_gui_adapter<T>(std::forward<Args...>(args)...));
        } catch (std::runtime_error& err) {
            show_error(std::string(err.what()));
        }
    }

    void show_error(const std::string& error);

    GROOT_APP_LOCAL void draw_gui();
    GROOT_APP_LOCAL void draw_command_gui();
    GROOT_APP_LOCAL void draw_background_tasks();
    GROOT_APP_LOCAL void draw_console_log();

    bool should_close();
    void step_gui();
    void step_gui(std::function<void(entt::registry&)> update);

    void main_loop();
    void main_loop(std::function<void(entt::registry&)> update);

private:
    entt::entity get_selected_entity();

    gfx::Gfx gui_app;
    entt::registry& registry;

    std::set<EntityEditor::ComponentTypeID> entity_filter;

    std::list<std::unique_ptr<Gui>> guis;

    Windows windows;

    std::shared_ptr<AppLog> app_log;
};
