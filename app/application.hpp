#pragma once

#include "command.hpp"
#include "command_gui.hpp"
#include "resources.hpp"
#include <gfx/gfx.hpp>
#include <list>
#include <async++.h>
#include "app_log.hpp"
#include "task.hpp"

class Application;


struct BackgroundTask {
    async::task<CommandState> task;
    std::unique_ptr<Command> command;
};

using BackgroundTaskHandle = std::list<BackgroundTask>::iterator;

struct Windows {
    bool background_tasks = true;
    bool history = true;
    bool main_viewer = true;
    bool console_log = true;
    bool demo_window = false;
};

class Application {
    friend class PlantGraphSource;

public:
    Application(entt::registry& reg);

    ~Application();

    BackgroundTaskHandle execute_command_async(Command* command);
    BackgroundTaskHandle execute_command_async(std::unique_ptr<Command>&& command);
    void execute_task(Task<void>&& t);

    template <typename R>
    void execute_task(Task<R>&& t)
    {
        execute_task(std::move(t.and_discard_result()));
    }


    void open_window(Gui* gui);

    template <typename T, typename... Args>
    void open_new_window(Args&&... args)
    {
        try {
            open_window(new T(std::forward<Args...>(args)...));
        } catch (std::runtime_error err) {
            show_error(std::string(err.what()));
        }
    }

    template <typename T, typename... Args>
    void open_new_window_adaptor(Args&&... args)
    {
        try {
            open_window(make_gui_adapter<T>(std::forward<Args...>(args)...));
        } catch (std::runtime_error err) {
            show_error(std::string(err.what()));
        }
    }

    void show_error(const std::string& error);

    void draw_gui();
    void draw_command_gui();
    void draw_background_tasks();
    void draw_console_log();

    void main_loop();

private:
    entt::entity get_selected_entity();

    gfx::Gfx gui_app;
    entt::registry& registry;

    std::set<EntityEditor::ComponentTypeID> entity_filter;

    std::list<std::unique_ptr<Gui>> guis;
    std::list<BackgroundTask> background_tasks;
    std::list<Task<void>> tasks;

    Windows windows;

    std::shared_ptr<AppLog> app_log;
};
