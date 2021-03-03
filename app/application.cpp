#include "application.hpp"
#include "create_graph.hpp"
#include "graph_cluster.hpp"
#include "open_graph.hpp"
#include "save_graph.hpp"
#include <gfx/imgui/gfx.hpp>
#include <gfx/imgui/imgui.h>
#include <jet/live/Live.hpp>

class JetListener : public jet::ILiveListener {
public:
    JetListener() { }

    void onLog(jet::LogSeverity severity, const std::string& message) override
    {
        switch (severity) {
        case jet::LogSeverity::kDebug:
            spdlog::debug("[Jet] {}", message);
            break;
        case jet::LogSeverity::kInfo:
            spdlog::info("[Jet] {}", message);
            break;
        case jet::LogSeverity::kWarning:
            spdlog::warn("[Jet] {}", message);
            break;
        case jet::LogSeverity::kError:
            spdlog::error("[Jet] {}", message);
            break;
        }
    }
    void onCodePreLoad() override { }
    void onCodePostLoad() override { }
};

jet::Live& jet_instance()
{
    static jet::Live live = jet::Live(std::make_unique<JetListener>());
    return live;
}

Application::Application()
    : gui_app(gfx::InitOptions {
        .title = "Groot Graph Viewer",
        .maximized = true,
        .resizable = true,
        .imgui = true,
        .debug_draw = true,
        .debug_context = true,
    })
    , plants(std::deque<Entry<groot::PlantGraph>>(1))
    , main_viewer(stack_top)
{
    stack_top.app = this;
    stack_push.app = this;

    stack_event.append([&]() {
        main_viewer.update_plant_graph();
    });
    init_lua();
}

void Application::init_lua()
{
    lua = luaL_newstate();
    luaL_openlibs(lua);
    lua_pushlightuserdata(lua, this);
    lua_setfield(lua, LUA_REGISTRYINDEX, "Groot_Application");



    static luaL_Reg funcs[] = {
        { "push_graph", [](lua_State* L) -> int {
            groot::PlantGraph* g = check_value<groot::PlantGraph>(L, 1); // 1
            
            lua_getfield(L, LUA_REGISTRYINDEX, "Groot_Application"); // 2
            Application* app = (Application*) lua_touserdata(L, 2);
            
            app->push_plant_graph(std::move(*g));
            lua_pushnil(L);
            lua_replace(L, 1);

            return 0;
        }},
        { nullptr, nullptr }
    };

    lua_newtable(lua);
    luaL_setfuncs(lua, funcs, 0);
    lua_setglobal(lua, "Groot");

    lua_open_graph(lua);
}

lua_State* Application::create_lua_context()
{
    return lua_newthread(lua);
}

void Application::execute_command_async(Command* command)
{
    BackgroundTask* task = new BackgroundTask;
    std::list<BackgroundTask*>::iterator it;
    {
        std::unique_lock _lock(background_task_lock);
        it = background_tasks.insert(background_tasks.end(), task);
    }

    task->command = command;

    task->task = std::async([=]() {
        CommandState result = command->execute();
        this->notify_task_finished(it, result);
    });
}

void Application::execute_command(Command* command)
{
    switch (command->execute()) {
    case CommandState::Error:
        this->show_error(command->error_string);
        break;

    default:
        break;
    }

    delete command;
}

void Application::notify_task_finished(std::list<BackgroundTask*>::iterator task, CommandState result)
{
    std::unique_lock _lock(background_task_lock);
    if (result == CommandState::Error) {
        this->show_error((*task)->command->error_string);
    }
    remove_background_tasks.push(*task);
    background_tasks.erase(task);
}

void Application::open_window(CommandGui* gui)
{
    std::unique_lock _lock(command_gui_lock);

    command_guis.push_back(gui);
}

void Application::show_error(const std::string& error)
{
    std::unique_lock _lock(error_lock);
    errors.push_back(error);
}

void Application::open_window(Editor&& editor)
{
    editors.emplace_back(std::move(editor));
}

void Application::draw_viewers()
{
    main_viewer.render();

    auto it = viewers.begin();
    while (it != viewers.end()) {
        if (!it->render()) {
            viewers.erase(it++);
        } else {
            it++;
        }
    }
}

void Application::draw_editors()
{

    auto it = editors.begin();
    while (it != editors.end()) {
        if (!it->render()) {
            editors.erase(it++);
        } else {
            it++;
        }
    }
}

void Application::draw_background_tasks()
{
    if (ImGui::Begin("Background tasks", &windows.background_tasks)) {
        std::shared_lock _lock(background_task_lock);
        for (auto it = background_tasks.begin(); it != background_tasks.end(); ++it) {
            ImGui::Spinner("##spinner", 10.0f);
            ImGui::SameLine();
            ImGui::Text("Background task %d", *it);
        }
    }
    ImGui::End();
}

void Application::draw_command_gui()
{
    std::unique_lock _lock(command_gui_lock);

    auto it = command_guis.begin();
    while (it != command_guis.end()) {
        switch ((*it)->draw_gui()) {
        case GuiState::Close:
            delete *it;
            command_guis.erase(it++);
            break;

        case GuiState::Editing:
            ++it;
            break;

        case GuiState::RunSync:
            this->execute_command(*it);
            command_guis.erase(it++);
            break;
        case GuiState::RunAsync:
            this->execute_command_async(*it);
            command_guis.erase(it++);
            break;
        }
    }
}

void Application::draw_history()
{
    if (ImGui::Begin("History", &windows.history)) {
        std::shared_lock _lock(plant_lock);

        for (size_t t = 0; t < plants.size(); t++) {
            ImGui::Text("Plant");
        }
    }
    ImGui::End();
}

void Application::draw_gui()
{
    ImGui::BeginMainWindow();

    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem(ICON_FA_FOLDER_OPEN "\tOpen Graph")) {
                open_window(new OpenGraph(stack_push));
            }

            if (ImGui::MenuItem(ICON_FA_SAVE "\tSave Graph")) {
                open_window(new SaveGraph(stack_top));
            }

            ImGui::Separator();

            if (ImGui::MenuItem(ICON_FA_FILE_IMPORT "\tImport PLY")) {
                open_window(new CreateGraph(stack_push));
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Edit")) {
            if (ImGui::MenuItem(ICON_FA_UNDO "\tUndo")) {
                pop_plant_graph();
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Process")) {
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tClustering...")) {
                open_window(new GraphCluster(stack_top, stack_push));
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Scripts")) {
            if (ImGui::MenuItem(ICON_FA_FOLDER_PLUS "\tNew script")) {
                open_window(std::move(Editor(this->create_lua_context())));
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Help")) {
            if (ImGui::MenuItem(ICON_FA_BUG "\tDemo window")) {
                windows.demo_window = true;
            }
            ImGui::EndMenu();
        }

        if (ImGui::MenuItem("Reload App")) {
            jet_instance().tryReload();
            spdlog::info("Reloading");
        }
        ImGui::EndMenuBar();
    }

    draw_viewers();
    draw_editors();
    draw_command_gui();
    draw_history();
    draw_background_tasks();

    if (windows.demo_window) ImGui::ShowDemoWindow(&windows.demo_window);

    ImGui::EndMainWindow();
    gui_app.draw_gui();
}

void Application::main_loop()
{
    gui_app.main_loop([&]() {
        jet_instance().update();

        while (!remove_background_tasks.empty()) {
            delete remove_background_tasks.front();
            remove_background_tasks.pop();
        }

        if (dirty_stack) {
            dirty_stack = false;
            main_viewer.update_plant_graph();
        }

        /*ImGui::Begin("New window");
        ImGui::End();*/
        draw_gui();
    });
}

void lua_add_tree(lua_State* L)
{
}
