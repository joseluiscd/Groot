#pragma once

#include "entity.hpp"
#include "python.hpp"
#include "python_task.hpp"
#include <groot_app/components.hpp>
#include <groot_app/workspace_io.hpp>

void create_registry_type(py::module_& m);

class AsyncViewer;

template <typename T>
inline py::object to_py_object(T&& obj)
{
    py::gil_scoped_acquire guard;
    return py::cast(obj);
}

inline auto to_py_entity(entt::registry& reg)
{
    return [&](entt::entity e) -> py::object {
        return py::cast(Entity(reg, e));
    };
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

    Entity new_entity()
    {
        return Entity(reg, reg.create());
    }

    AsyncViewer* create_viewer();
    /// @return None
    PythonTask load(const std::string& filename)
    {
        return PythonTask {
            open_workspace_command(reg, filename)
        };
    }

    /// @return None
    PythonTask save(const std::string& filename)
    {
        return PythonTask {
            save_workspace_command(reg, filename)
        };
    }

    /// @return Entity
    PythonTask load_ply(const std::string& filename)
    {
        return PythonTask {
            import_ply_command(reg, filename)
                .then(sync_scheduler(), to_py_entity(reg))
        };
    }

    /// @return Entity
    PythonTask load_graph(const std::string& filename)
    {
        return PythonTask {
            import_graph_command(reg, filename)
                .then(sync_scheduler(), to_py_entity(reg))
        };
    }

    void run_viewer(py::object init_func, py::object update_func)
    {
        Application app(reg);

        {
            py::gil_scoped_acquire guard;
            if (!init_func.is_none()) {
                init_func(this);
            }
        }

        ImGuiContext* ctx = reg.ctx<ImGuiContext*>();
        ImGui::SetCurrentContext(ctx);

        bool update_func_none;
        {
            py::gil_scoped_acquire guard;
            update_func_none = update_func.is_none();
        }

        if (update_func_none) {
            app.main_loop();
        } else {
            app.main_loop([this, &update_func](entt::registry&) {
                py::gil_scoped_acquire guard;
                update_func(this);
            });
        }
    }

    void schedule_task(PythonTask& t, const std::string& name)
    {
        reg.ctx<TaskManager>().push_task(name, t.ignore_result());
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
                py::gil_scoped_acquire guard;
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