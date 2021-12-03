#pragma once

#include "python.hpp"
#include "task.hpp"
#include <optional>

enum TaskMode {
    Async = 0,
    Sync,
    Inline
};

struct PythonThreadState {
    static void create_if_needed()
    {
        if (state == nullptr) {
            state = PyThreadState_New(PyInterpreterState_Get());
        }
    }

    static void release_thread()
    {
        state = PyEval_SaveThread();
    }

    static void acquire_thread()
    {
        create_if_needed();
        PyEval_RestoreThread(state);
    }

    static thread_local PyThreadState* state;
};

struct ReleaseGilGuard {
    ReleaseGilGuard()
    {
        PythonThreadState::release_thread();
    }

    ~ReleaseGilGuard()
    {
        PythonThreadState::acquire_thread();
    }
};

struct AcquireGilGuard {
    AcquireGilGuard()
        : state(PyGILState_Ensure())
    {
    }

    ~AcquireGilGuard()
    {
        PyGILState_Release(state);
    }

    PyGILState_STATE state;
};

template <typename T>
struct DestroyWithGil {
    DestroyWithGil(const T& _obj)
        : obj(_obj)
    {
    }

    ~DestroyWithGil()
    {
        AcquireGilGuard guard;
        obj.reset();
    }

    std::optional<T> obj;
};

template <typename T>
DestroyWithGil<T> destroy_with_gil(const T& obj)
{
    return DestroyWithGil<T>(obj);
}

inline auto call_f(boost::python::object f)
{
    return [f = destroy_with_gil(f)](boost::python::object result) {
        AcquireGilGuard guard;
        boost::python::object function = *f.obj;
        return std::invoke(function, result);
    };
}

class PythonTask : public async::task<boost::python::object> {
public:
    PythonTask(async::task<boost::python::object>&& _t, boost::python::str _name)
        : async::task<boost::python::object>(std::move(_t))
        , name(_name)
    {
    }

    PythonTask(async::task<void>&& _t, boost::python::str _name)
        : async::task<boost::python::object>(std::move(_t.then(async::inline_scheduler(), create_none)))
        , name(_name)
    {
    }

    template <typename Result>
    PythonTask(async::task<boost::python::object>&& _t, boost::python::str _name)
        : async::task<boost::python::object>(std::move(_t.then(async::inline_scheduler(), [](Result&& r) {
            return boost::python::object(std::forward<Result>(r));
        })))
        , name(_name)
    {
    }

    PythonTask(boost::python::object func, boost::python::str _name, TaskMode mode)
        : async::task<boost::python::object>(std::move(spawn(func, mode)))
        , name(_name)
    {
    }

    template <typename Result>
    Result get_extract()
    {
        return boost::python::extract<Result>(this->get());
    }

    async::task<void> ignore_result()
    {
        return this->then(async::inline_scheduler(), []() {});
    }

    static boost::python::object create_none()
    {
        return boost::python::object();
    }

    void then_python(boost::python::object f, TaskMode mode)
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
    }

    void run_till_completion();

    boost::python::str name;

private:
    static async::task<boost::python::object> spawn_async(boost::python::object f)
    {
        return async::spawn(async_scheduler(), f);
    }

    static async::task<boost::python::object> spawn_sync(boost::python::object f)
    {
        return async::spawn(sync_scheduler(), f);
    }

    static async::task<boost::python::object> spawn_inline(boost::python::object f)
    {
        return async::spawn(sync_scheduler(), f);
    }

    static async::task<boost::python::object> spawn(boost::python::object f, TaskMode mode)
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

void create_task_module();