#pragma once

#include <optional>
#include <pybind11/pybind11.h>
#include <entt/meta/meta.hpp>

namespace py = pybind11;

template <typename T>
struct DestroyWithGil {
    DestroyWithGil(const T& _obj)
        : obj(_obj)
    {
    }

    ~DestroyWithGil()
    {
        py::gil_scoped_acquire guard;
        obj.reset();
    }

    std::optional<T> obj;
};

template <typename T>
DestroyWithGil<T> destroy_with_gil(const T& obj)
{
    return DestroyWithGil<T>(obj);
}

py::object any_to_python(py::object parent, void* component, const entt::type_info& type);

void create_pygroot_module(py::module_& m);