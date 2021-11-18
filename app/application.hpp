#pragma once

#include "command.hpp"
#include "command_gui.hpp"
#include "resources.hpp"
#include <future>
#include <gfx/gfx.hpp>
#include <list>
#include <queue>
#include <shared_mutex>
#include <async++.h>
#include "app_log.hpp"

class Application;


struct BackgroundTask {
    async::task<CommandState> task;
    std::unique_ptr<Command> command;
};

using BackgroundTaskHandle = std::list<std::shared_ptr<BackgroundTask>>::iterator;

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
    std::list<std::shared_ptr<BackgroundTask>> background_tasks;
    std::queue<std::shared_ptr<BackgroundTask>> remove_background_tasks;

    std::shared_mutex background_task_lock;
    std::shared_mutex command_gui_lock;

    Windows windows;

    std::shared_ptr<AppLog> app_log;
};
