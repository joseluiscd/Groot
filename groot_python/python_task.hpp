#pragma once

#include "python.hpp"
#include <groot/groot.hpp>
#include <groot_app/task.hpp>
#include <optional>

enum TaskMode {
    Async = 0,
    Sync,
    Inline
};

/// Call python function f, without the GIL complaining with the destructor.
inline auto call_python_function(py::object f)
{
    return [f = destroy_with_gil(f)](py::object result) {
        py::gil_scoped_acquire guard;
        py::object function = *f.obj;
        return std::invoke(function, result);
    };
}

class PythonTask : public async::task<py::object> {
public:
    PythonTask(async::task<py::object>&& _t)
        : async::task<py::object>(std::move(_t))
    {
    }

    PythonTask(async::task<void>&& _t)
        : async::task<py::object>(_t.then(async::inline_scheduler(), create_none))
    {
    }

    template <typename Result>
    PythonTask(async::task<py::object>&& _t)
        : async::task<py::object>(std::move(_t.then(async::inline_scheduler(), [](Result&& r) {
            return py::object(std::forward<Result>(r));
        })))
    {
    }

    PythonTask(py::object func, TaskMode mode)
        : async::task<py::object>(spawn(func, mode))
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
        return py::none();
    }

    PythonTask& then_python(py::object f, TaskMode mode)
    {
        switch (mode) {
        case TaskMode::Async:
            *this = PythonTask(this->then(async_scheduler(), call_python_function(f)));
            break;
        case TaskMode::Sync:
            *this = PythonTask(this->then(sync_scheduler(), call_python_function(f)));
            break;
        case TaskMode::Inline:
            *this = PythonTask(this->then(async::inline_scheduler(), call_python_function(f)));
            break;
        default:
            *this = PythonTask(this->then(sync_scheduler(), call_python_function(f)));
            break;
        };
        return *this;
    }

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