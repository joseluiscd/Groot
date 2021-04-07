#include "application.hpp"
#include "create_graph.hpp"
#include "graph_cluster.hpp"
#include "open_graph.hpp"
//#include "particle_sim.hpp"
#include "graph_viewer_system.hpp"
#include "save_graph.hpp"
#include "viewer_system.hpp"
#include <gfx/imgui/gfx.hpp>
#include <gfx/imgui/imgui.h>
#include <gfx/render_pass.hpp>

#ifdef HOT_CODE_RELOAD
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
#endif

Application::Application()
    : gui_app(gfx::InitOptions {
        .title = "Groot Graph Viewer",
        .maximized = true,
        .resizable = true,
        .imgui = true,
        .debug_draw = true,
        .debug_context = true,
    })
{
    viewer_system::init(registry);
    graph_viewer_system::init(registry);

    init_lua();
}

Application::~Application()
{
    lua_close(lua);
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
             Application* app = (Application*)lua_touserdata(L, 2);

             //app->push_plant_graph(std::move(*g));
             lua_pushnil(L);
             lua_replace(L, 1);

             return 0;
         } },
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

BackgroundTaskHandle Application::execute_command_async(std::unique_ptr<Command>&& command)
{
    BackgroundTaskHandle it;

    {
        std::unique_lock _lock(background_task_lock);
        it = background_tasks.emplace(background_tasks.end(), std::make_shared<BackgroundTask>());
    }

    (*it)->command = std::move(command);
    (*it)->task = std::async([=]() {
        std::shared_lock _lock((*it)->lock);

        CommandState result = (*it)->command->execute();
        this->notify_task_finished(it, result);
    });

    return it;
}

BackgroundTaskHandle Application::execute_command_async(Command* command)
{
    return execute_command_async(std::unique_ptr<Command>(command));
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

void Application::notify_task_finished(BackgroundTaskHandle task, CommandState result)
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

    command_guis.emplace_back(gui);
}

void Application::show_error(const std::string& error)
{
    std::unique_lock _lock(error_lock);
    errors.push_back(error);
}

void Application::open_window(Editor* editor)
{
    editors.emplace_back(editor);
}

void Application::draw_editors()
{

    auto it = editors.begin();
    while (it != editors.end()) {
        if (!(*it)->render()) {
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
            ImGui::Text("Background task %ld", (size_t)it->get());
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
            command_guis.erase(it++);
            break;

        case GuiState::Editing:
            ++it;
            break;

        case GuiState::RunSync:
            this->execute_command(it->get());
            command_guis.erase(it++);
            break;
        case GuiState::RunAsync:
            this->execute_command_async(std::move(*it));
            command_guis.erase(it++);
            break;
        case GuiState::RunAsyncUpdate:
            this->execute_command_async(std::move(*it));
            ++it;
            break;
        }
    }
}

void Application::draw_gui()
{
    ImGui::BeginMainWindow();

    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem(ICON_FA_FOLDER_OPEN "\tOpen Graph")) {
                open_window(new OpenGraph(registry));
            }

            if (ImGui::MenuItem(ICON_FA_SAVE "\tSave Graph")) {
                //open_window(new SaveGraph(stack_top));
            }

            ImGui::Separator();

            if (ImGui::MenuItem(ICON_FA_FILE_IMPORT "\tImport PLY")) {
                open_window(new CreateGraph(registry));
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Process")) {
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tClustering...")) {
                //open_window(new GraphCluster(stack_top, stack_push));
            }
            if (ImGui::MenuItem(ICON_FA_PARACHUTE_BOX "\tParticle simulator...")) {
                //open_window(new ParticleSim());
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Scripts")) {
            if (ImGui::MenuItem(ICON_FA_FOLDER_PLUS "\tNew script")) {
                open_window(new Editor(this->create_lua_context()));
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Help")) {
            if (ImGui::MenuItem(ICON_FA_BUG "\tDemo window")) {
                windows.demo_window = true;
            }
            ImGui::EndMenu();
        }

#ifdef HOT_CODE_RELOAD
        if (ImGui::MenuItem("Reload App")) {
            jet_instance().tryReload();
            spdlog::info("Reloading");
        }
#endif
        ImGui::EndMenuBar();
    }

    {
        auto& fbo = registry.ctx<viewer_system::SystemData>().framebuffer;
        gfx::RenderPass(fbo, gfx::ClearOperation::color_and_depth());
    }

    graph_viewer_system::run(registry);
    viewer_system::run(registry);

    draw_editors();
    draw_command_gui();
    draw_background_tasks();

    if (windows.demo_window)
        ImGui::ShowDemoWindow(&windows.demo_window);

    ImGui::EndMainWindow();
    gui_app.draw_gui();
}

void Application::main_loop()
{
    gui_app.main_loop([&]() {
#ifdef HOT_CODE_RELOAD
        jet_instance().update();
#endif

        while (!remove_background_tasks.empty()) {
            remove_background_tasks.pop();
        }

        draw_gui();
    });
}

void lua_add_tree(lua_State* L)
{
}
