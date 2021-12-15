#pragma once

#include "entity.hpp"
#include "python.hpp"
#include "python_task.hpp"
#include <groot_app/components.hpp>
#include <groot_app/open_workspace.hpp>
#include <groot_app/save_workspace.hpp>

void create_registry_type(py::module_& m);

class Registry {
public:
    static entt::type_info type_id;

    Registry()
        : reg()
    {
    }

    entt::handle selected()
    {
        return entt::handle(reg, reg.ctx<SelectedEntity>().selected);
    }

    py::list entities()
    {
        py::list handles;
        reg.each([&](auto entity) {
            handles.append(Entity(reg, entity));
        });

        return handles;
    }

    void load(const std::string& filename)
    {
        ReleaseGilGuard guard;

        OpenWorkspace cmd;
        cmd.set_file(filename);
        cmd.run(reg);
    }

    void save(const std::string& filename)
    {
        ReleaseGilGuard guard;
        SaveWorkspace cmd { reg };
        cmd.set_file(filename);
        cmd.run(reg);
    }

    Entity load_ply(const std::string& filename)
    {
        ReleaseGilGuard guard;
        auto&& task = import_ply_command(reg, filename);

        return Entity(reg, run_task(std::move(task)));
    }

    Entity load_graph(const std::string& filename)
    {
        ReleaseGilGuard guard;
        auto&& task = import_graph_command(reg, filename);

        return Entity(reg, run_task(std::move(task)));
    }

    Entity new_entity()
    {
        return Entity(reg, reg.create());
    }

    void run_viewer(py::object init_func, py::object update_func)
    {
        Application app(reg);

        {
            AcquireGilGuard guard;
            if (!init_func.is_none()) {
                init_func(this);
            }
        }

        ImGuiContext* ctx = reg.ctx<ImGuiContext*>();
        ImGui::SetCurrentContext(ctx);

        bool update_func_none;
        {
            AcquireGilGuard guard;
            update_func_none = update_func.is_none();
        }

        if (update_func_none) {
            app.main_loop();
        } else {
            app.main_loop([this, &update_func](entt::registry&) {
                AcquireGilGuard guard;
                update_func(this);
            });
        }
    }

    void run_tasks()
    {
        sync_scheduler().run_all_tasks();
    }

    void schedule_task(PythonTask& t)
    {
        std::string name = py::cast<std::string>(t.name);
        reg.ctx<TaskBroker>().push_task(name, std::move(t.ignore_result()));
    }

    entt::registry reg;
};
