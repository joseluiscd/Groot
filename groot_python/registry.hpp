#pragma once

#include <groot_app/components.hpp>
#include <groot_app/open_workspace.hpp>
#include <groot_app/save_workspace.hpp>
#include "python.hpp"
#include "python_task.hpp"
#include "entity.hpp"

void create_registry_type();

class Registry {
public:
    Registry()
        : reg()
    {
        reg.prepare<Visible>();

        reg.prepare<groot::PlantGraph>();
        reg.prepare<PointCloud>();
        reg.prepare<PointNormals>();
        reg.prepare<PointColors>();
        reg.prepare<Cylinders>();
    }

    entt::handle selected()
    {
        return entt::handle(reg, reg.ctx<SelectedEntity>().selected);
    }

    boost::python::list entities()
    {
        boost::python::list handles;
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

    void run_viewer(boost::python::object init_func, boost::python::object update_func)
    {
        Application app(reg);
        {
            auto state = PyGILState_Ensure();
            std::invoke(init_func, boost::ref(*this));
            PyGILState_Release(state);
        }

        app.main_loop([this, &update_func](entt::registry&) {
            std::invoke(update_func, boost::ref(*this));
        });
    }

    void run_tasks()
    {
        sync_scheduler().run_all_tasks();
    }

    void schedule_task(PythonTask& t)
    {
        std::string name = boost::python::extract<std::string>(t.name);
        reg.ctx<TaskBroker>().push_task(name, std::move(t.ignore_result()));
    }

private:
    entt::registry reg;
};
