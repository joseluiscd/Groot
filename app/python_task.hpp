#pragma once

#include "python.hpp"
#include "task.hpp"

enum TaskMode {
    Async = 0,
    Sync,
    Inline
};



struct PythonThreadState {
    PythonThreadState()
        : state(nullptr)
    {
    }

    void create_if_needed()
    {
        if (state == nullptr) {
            state = PyThreadState_New(PyInterpreterState_Get());
        }
    }

    void release_gil()
    {
        create_if_needed();
        state = PyEval_SaveThread();
    }

    void acquire_gil()
    {
        create_if_needed();
        PyEval_RestoreThread(state);
    }

    ~PythonThreadState()
    {
        if (state != nullptr) {
            PyThreadState_Clear(state);
        }
    }

    PyThreadState* state;
};


struct ReleaseGil : boost::python::default_call_policies {
    template <class ArgumentPackage>
    static bool precall(ArgumentPackage const&)
    {
        thread_state.create_if_needed();
        thread_state.release_gil();
        return true;
    }

    // Pass the result through
    template <class ArgumentPackage>
    static PyObject* postcall(ArgumentPackage const&, PyObject* result)
    {
        thread_state.acquire_gil();
        return result;
    }

    // Retain pointer to PyThreadState on a per-thread basis here
    static thread_local PythonThreadState thread_state;
};

struct ReleaseGilGuard {
    ReleaseGilGuard() {
        ReleaseGil::thread_state.release_gil();
    }

    ~ReleaseGilGuard()
    {
        ReleaseGil::thread_state.acquire_gil();
    }
};

struct AcquireGilGuard {
    AcquireGilGuard() {
        ReleaseGil::thread_state.acquire_gil();
    }

    ~AcquireGilGuard()
    {
        ReleaseGil::thread_state.release_gil();
    }
};

inline auto call_f(boost::python::object f)
{
    return [f](boost::python::object result) {
        AcquireGilGuard guard;
        return std::invoke(f, result);
    };
}

class PythonTask : public async::task<boost::python::object> {
public:
    PythonTask(async::task<boost::python::object>&& _t, boost::python::str _name)
        : async::task<boost::python::object>(std::move(_t))
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

    boost::python::object run_in_pythread(boost::python::object o);

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