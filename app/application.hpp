#pragma once

#include "command.hpp"
#include "command_gui.hpp"
#include "editor.hpp"
#include "lua.hpp"
#include <future>
#include <gfx/gfx.hpp>
#include <groot/plant_graph.hpp>
#include <list>
#include <mutex>
#include <queue>
#include <stack>
#include <entt/entt.hpp>
#include "resources.hpp"

class Application;

struct BackgroundTask {
    std::shared_mutex lock;

    std::future<void> task;
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
    Application();
    ~Application();

    void execute_command(Command* command);
    void execute_command(std::unique_ptr<Command> command) { execute_command(command.get()); }

    BackgroundTaskHandle execute_command_async(Command* command);
    BackgroundTaskHandle execute_command_async(std::unique_ptr<Command>&& command);

    void open_window(CommandGui* gui);
    void open_window(Editor* editor);

    template <typename T, typename... Args>
    void open_new_window(Args&&... args) {
        try {
            open_window(new T(args...));
        } catch (std::runtime_error err) {
            show_error(std::string(err.what()));
        }
    }

    void init_lua();
    lua_State* create_lua_context();

    void notify_task_finished(BackgroundTaskHandle task, CommandState result);

    void show_error(const std::string& error);

    void draw_gui();
    void draw_command_gui();
    void draw_editors();
    void draw_background_tasks();

    lua_State* create_context();

    void main_loop();


private:
    entt::entity get_selected_entity();

    gfx::Gfx gui_app;
    entt::registry registry;

    std::set<EntityEditor::ComponentTypeID> entity_filter;

    std::list<std::unique_ptr<CommandGui>> command_guis;
    std::list<std::shared_ptr<BackgroundTask>> background_tasks;
    std::queue<std::shared_ptr<BackgroundTask>> remove_background_tasks;
    std::list<std::unique_ptr<Editor>> editors;

    std::shared_mutex background_task_lock;
    std::shared_mutex command_gui_lock;

    Windows windows;
    lua_State* lua;
};
