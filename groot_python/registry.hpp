#pragma once

#include "entity.hpp"
#include "python.hpp"
#include "python_task.hpp"
#include <groot_app/components.hpp>
#include <groot_app/open_workspace.hpp>
#include <groot_app/save_workspace.hpp>

void create_registry_type(py::module_& m);

class AsyncViewer;

inline auto create_entity(entt::registry& reg)
{
    return [&](entt::entity e) {
        return Entity(reg, e);
    };
}

template <typename T>
inline py::object convert_python(T&& obj)
{
    AcquireGilGuard guard;
    return py::cast(obj);
}

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

        run_task(open_workspace_command(reg, filename));
    }

    void save(const std::string& filename)
    {
        ReleaseGilGuard guard;
        SaveWorkspace cmd { reg };
        cmd.set_file(filename);
        cmd.run(reg);
    }

    PythonTask load_ply(const std::string& filename)
    {
        ReleaseGilGuard guard;
        return PythonTask {
            import_ply_command(reg, filename)
                .then(create_entity(reg))
                .then(convert_python<Entity>)
        };
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

    AsyncViewer* create_viewer();

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

    void schedule_task(PythonTask& t, const std::string& name)
    {
        reg.ctx<TaskBroker>().push_task(name, t.ignore_result());
    }

    entt::registry reg;
};

class AsyncViewer {
public:
    AsyncViewer(Registry& _reg)
        : app(_reg.reg)
        , reg(_reg)
    {
        ImGuiContext* ctx = reg.reg.ctx<ImGuiContext*>();
        ImGui::SetCurrentContext(ctx);
    }

    void step(py::object update_func)
    {
        if (update_func.is_none()) {
            app.step_gui();
        } else {
            app.step_gui([&](entt::registry&) {
                AcquireGilGuard guard;
                update_func(reg);
            });
        }
    }

    bool should_close()
    {
        return app.should_close();
    }

private:
    Application app;
    Registry& reg;
};

inline AsyncViewer* Registry::create_viewer()
{
    return new AsyncViewer { *this };
}