#pragma once

#include <groot/groot.hpp>
#include "python.hpp"
#include <groot_app/task.hpp>
#include <optional>

enum TaskMode {
    Async = 0,
    Sync,
    Inline
};

inline auto call_f(py::object f)
{
    return [f = destroy_with_gil(f)](py::object result) {
        AcquireGilGuard guard;
        py::object function = *f.obj;
        return std::invoke(function, result);
    };
}

class GROOT_API PythonTask : public async::task<py::object> {
public:
    PythonTask(async::task<py::object>&& _t, py::str _name)
        : async::task<py::object>(std::move(_t))
        , name(_name)
    {
    }

    PythonTask(async::task<void>&& _t, py::str _name)
        : async::task<py::object>(std::move(_t.then(async::inline_scheduler(), create_none)))
        , name(_name)
    {
    }

    template <typename Result>
    PythonTask(async::task<py::object>&& _t, py::str _name)
        : async::task<py::object>(std::move(_t.then(async::inline_scheduler(), [](Result&& r) {
            return py::object(std::forward<Result>(r));
        })))
        , name(_name)
    {
    }

    PythonTask(py::object func, py::str _name, TaskMode mode)
        : async::task<py::object>(std::move(spawn(func, mode)))
        , name(_name)
    {
    }

    template <typename Result>
    Result get_extract()
    {
        return this->get().cast<Result>();
    }

    async::task<void> ignore_result()
    {
        return this->then(async::inline_scheduler(), []() {});
    }

    static py::object create_none()
    {
        return py::object();
    }

    PythonTask& then_python(py::object f, TaskMode mode)
    {
        switch (mode) {
        case TaskMode::Async:
            *this = PythonTask(std::move(this->then(async_scheduler(), call_f(f))), this->name);
            break;
        case TaskMode::Sync:
            *this = PythonTask(std::move(this->then(sync_scheduler(), call_f(f))), this->name);
            break;
        case TaskMode::Inline:
            *this = PythonTask(std::move(this->then(async::inline_scheduler(), call_f(f))), this->name);
            break;
        default:
            *this = PythonTask(std::move(this->then(sync_scheduler(), call_f(f))), this->name);
            break;
        };
        return *this;
    }

    void run_till_completion();

    py::str name;

private:
    static async::task<py::object> spawn_async(py::object f)
    {
        return async::spawn(async_scheduler(), f);
    }

    static async::task<py::object> spawn_sync(py::object f)
    {
        return async::spawn(sync_scheduler(), f);
    }

    static async::task<py::object> spawn_inline(py::object f)
    {
        return async::spawn(sync_scheduler(), f);
    }

    static async::task<py::object> spawn(py::object f, TaskMode mode)
    {
        switch (mode) {
        case TaskMode::Async:
            return spawn_async(f);
        case TaskMode::Sync:
            return spawn_sync(f);
        case TaskMode::Inline:
            return spawn_inline(f);
        default:
            return spawn_sync(f);
        }
    }
};

void create_task_module(py::module_& m);