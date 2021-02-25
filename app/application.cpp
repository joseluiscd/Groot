#include "application.hpp"
#include "create_graph.hpp"
#include "open_graph.hpp"
#include "graph_cluster.hpp"
#include <gfx/imgui/gfx.hpp>
#include <gfx/imgui/imgui.h>

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
    stack_event.append([&](){
        main_viewer.update_plant_graph();
    });

}

void Application::execute_command_async(Command* command)
{
    std::unique_lock _lock(background_task_lock);

    BackgroundTask* task = new BackgroundTask;
    auto it = background_tasks.insert(background_tasks.end(), task);

    task->command = command;
    command->app = this;

    task->task = std::async([=]() {
        CommandState result = command->execute();
        this->notify_task_finished(it, result);
    });
}

void Application::execute_command(Command* command)
{
    command->app = this;

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

void Application::draw_viewers()
{
    main_viewer.render();

    auto it = viewers.begin();
    while (it != viewers.end()) {
        if (! it->render()) {
            viewers.erase(it++);
        } else {
            it++;
        }
    }
}

void Application::draw_background_tasks()
{
    if(ImGui::Begin("Background tasks", &windows.background_tasks)) {
        std::shared_lock _lock(background_task_lock);
        auto it = background_tasks.begin();
        while (it != background_tasks.end()) {
            ImGui::Text("Background task %d", &*it);
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
    while (!remove_background_tasks.empty()) {
        delete remove_background_tasks.front();
        remove_background_tasks.pop();
    }

    if (dirty_stack) {
        dirty_stack = false;
        main_viewer.update_plant_graph();
    }

    ImGui::BeginMainWindow();

    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem(ICON_FA_FILE_IMPORT "\tOpen PLY")) {
                open_window(new CreateGraph());
            }

            if (ImGui::MenuItem(ICON_FA_FOLDER_OPEN "\tOpen Graph")) {
                open_window(new OpenGraph());
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Process")) {
            if (ImGui::MenuItem(ICON_FA_CALCULATOR "\tClustering...")) {
                open_window(new GraphCluster(stack_top));
            }
            ImGui::EndMenu();
        }

        if (ImGui::MenuItem("Reload App")) {
            live_instance.tryReload();
            spdlog::info("Reloading");
        }
        ImGui::EndMenuBar();
    }

    draw_viewers();
    draw_command_gui();
    draw_history();
    draw_background_tasks();

    ImGui::EndMainWindow();
    gui_app.draw_gui();
}

void Application::main_loop()
{
    gui_app.main_loop([&]() {
        live_instance.update();
        /*ImGui::Begin("MIau");
        ImGui::End();*/
        draw_gui();
    });
}
