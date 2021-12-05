#pragma once

#include <boost/python.hpp>


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

