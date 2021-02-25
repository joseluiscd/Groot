#pragma once

#include "data_source.hpp"
#include "graph_view.hpp"
#include <eventpp/callbacklist.h>
#include <future>
#include <gfx/gfx.hpp>
#include <groot/plant_graph.hpp>
#include <list>
#include <mutex>
#include <queue>
#include <stack>
#include <jet/live/Live.hpp>

class Application;

enum class CommandState : bool {
    Ok = false,
    Error = true,
};

/// Abstract command
class Command {
public:
    virtual CommandState execute() = 0;

    Application* app;
    std::string error_string = "";
};

enum class GuiState {
    Close = 0,
    Editing,
    RunAsync,
    RunSync,
};

template <typename T>
class Entry : public IDataSource<T> {
public:
    Entry()
        : data()
    {
    }

    Entry(T&& _data)
        : data(std::move(_data))
    {
    }

    ~Entry()
    {
        observers();
    }

    T& operator*() override { return data; }
    const T& operator*() const override { return data; }

    eventpp::CallbackList<IDataSourceDestroyed> observers;

private:
    T data;
};

class CommandGui : public Command {
public:
    virtual GuiState draw_gui() = 0;
};

struct BackgroundTask {
    std::future<void> task;
    Command* command;
};

struct Windows {
    bool background_tasks = true;
    bool history = true;
    bool main_viewer = true;
    bool console_log = true;
};

class Application {
    friend class PlantGraphSource;

public:
    Application();

    void execute_command(Command* command);
    void execute_command_async(Command* command);
    void execute_command(Command& command);
    void execute_command_async(Command& command);
    void open_window(CommandGui& gui);
    void open_window(CommandGui* gui);
    void open_window(GraphViewer* view);

    void notify_task_finished(std::list<BackgroundTask*>::iterator task, CommandState result);

    void show_error(const std::string& error);

    void draw_gui();
    void draw_command_gui();
    void draw_viewers();
    void draw_background_tasks();
    void draw_history();

    void main_loop();

    /// Thread-safe
    void push_plant_graph(groot::PlantGraph&& g)
    {
        std::unique_lock _lock(plant_lock);
        plants.emplace(Entry<groot::PlantGraph>(std::move(g)));
        dirty_stack = true;
    }

private:
    class PlantGraphSource : public IDataSource<groot::PlantGraph> {
    public:
        groot::PlantGraph& operator*() { return *app->plants.top(); }
        const groot::PlantGraph& operator*() const { return *app->plants.top(); }

        Application* app;
    };
    gfx::Gfx gui_app;
    jet::Live live_instance;

    std::stack<Entry<groot::PlantGraph>> plants;
    groot::PlantGraph* selected;
    PlantGraphSource stack_top;
    bool dirty_stack = false;

    GraphViewer main_viewer;

    std::list<CommandGui*> command_guis;
    std::list<BackgroundTask*> background_tasks;
    std::queue<BackgroundTask*> remove_background_tasks;
    std::list<std::string> errors;
    std::list<GraphViewer> viewers;

    std::shared_mutex background_task_lock;
    std::shared_mutex command_gui_lock;
    std::shared_mutex command_lock;
    std::shared_mutex plant_lock;
    std::shared_mutex error_lock;

    Windows windows;

    eventpp::CallbackList<IDataSourceChanged<groot::PlantGraph>> stack_event;
};
