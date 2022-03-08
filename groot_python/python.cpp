#include "python.hpp"
#include "entity.hpp"
#include "python_graph.hpp"
#include "python_imgui.hpp"
#include "python_task.hpp"
#include "registry.hpp"

#include "groot/cgal.hpp"
#include "groot/cloud.hpp"
#include <boost/core/noncopyable.hpp>
#include <entt/meta/pointer.hpp>
#include <functional>
#include <groot_app/entt.hpp>
#include <groot_graph/plant_graph_compare.hpp>
#include <pybind11/numpy.h>

using namespace entt::literals;

constexpr const entt::id_type convert_to_python = "convert_to_python"_hs;
constexpr const entt::id_type convert_from_python = "convert_from_python"_hs;

#define RETURN_NONE_IF_NOT(EXPR)    \
    if (!static_cast<bool>(EXPR)) { \
        return py::none();          \
    }
#define RETURN_NULL_IF_NOT(EXPR)    \
    if (!static_cast<bool>(EXPR)) { \
        return nullptr;             \
    }

/*
struct dynamic_from_python {
    dynamic_from_python()
    {
        boost::python::converter::registry::push_back(&convertible, &construct, boost::python::type_id<DynamicComponent>());
    }

    static void* convertible(PyObject* obj)
    {
        PyObject* type = PyObject_GetAttrString(obj, "type_id");
        RETURN_NULL_IF_NOT(type);

        boost::python::extract<entt::type_info> info_extract(type);
        RETURN_NULL_IF_NOT(info_extract.check());
        entt::type_info info = info_extract;

        boost::python::object object { boost::python::handle<>(obj) };
        entt::meta_type meta_type = entt::resolve(info);
        RETURN_NULL_IF_NOT(meta_type);

        entt::meta_func convert_f = meta_type.func(convert_from_python);
        RETURN_NULL_IF_NOT(convert_f);

        RETURN_NULL_IF_NOT(convert_f.arity() == 1);
        RETURN_NULL_IF_NOT(convert_f.is_static());
        RETURN_NULL_IF_NOT(convert_f.arg(0) == entt::resolve<boost::python::object>());

        entt::meta_any constructed = convert_f.invoke(entt::meta_handle(), object);
        RETURN_NULL_IF_NOT(constructed);

        DynamicComponent dc = constructed.cast<DynamicComponent>();
        return dc.second;
    }

    static void construct(PyObject* obj, boost::python::converter::rvalue_from_python_stage1_data* data)
    {
        void* storage = ((boost::python::converter::rvalue_from_python_storage<DynamicComponent>*)data)->storage.bytes;
        void* object = data->convertible;

        PyObject* type = PyObject_GetAttrString(obj, "type_id");

        entt::type_info info = boost::python::extract<entt::type_info>(type);

        storage = new DynamicComponent(info, object);
    }
};*/

template <typename Component>
py::object component_to_python(void* c, py::object parent)
{
    return py::cast(static_cast<Component*>(c), py::return_value_policy::reference_internal, parent);
}

template <typename Component>
Component& component_from_python(py::object& o)
{
    return py::cast<Component&>(o);
}

template <>
struct entt::is_equality_comparable<py::object> {
    static constexpr const bool value = false;
};

py::object any_to_python(py::object parent, void* component, const entt::type_info& type)
{
    entt::meta_type meta_type = entt::resolve(type);

    RETURN_NONE_IF_NOT(meta_type);

    auto function = meta_type.func(convert_to_python);
    RETURN_NONE_IF_NOT((bool)function);
    RETURN_NONE_IF_NOT(function.arity() == 2);
    RETURN_NONE_IF_NOT(function.is_static());
    RETURN_NONE_IF_NOT(function.ret() == entt::resolve<py::object>());

    entt::meta_any result_any = function.invoke(entt::meta_handle(), component, parent);
    RETURN_NONE_IF_NOT(result_any);
    RETURN_NONE_IF_NOT(result_any.allow_cast<py::object>());

    return result_any.cast<py::object>();
}

template <typename Component>
void declare_python_component(py::module_& m, const std::string_view& name)
{
    entt::meta<Component>()
        .type()
        .template func<component_to_python<Component>>(convert_to_python)
        .template func<component_from_python<Component>, entt::as_ref_t>(convert_from_python);

    m.add_object(name.data(), py::cast(entt::type_id<Component>()));
}

py::buffer_info create_buffer_info(std::vector<groot::Vector_3>& points)
{
    return py::buffer_info(
        (void*)points.data(),
        sizeof(float),
        py::format_descriptor<float>::format(),
        2,
        py::array::ShapeContainer({ points.size(), (size_t)3 }),
        py::array::ShapeContainer({ sizeof(float) * 3, sizeof(float) }));
}

py::buffer_info create_buffer_info(std::vector<groot::Point_3>& points)
{
    return py::buffer_info(
        (void*)points.data(),
        sizeof(float),
        py::format_descriptor<float>::format(),
        2,
        py::array::ShapeContainer({ points.size(), (size_t)3 }),
        py::array::ShapeContainer({ sizeof(float) * 3, sizeof(float) }));
}

void create_pygroot_module(py::module_& m)
{
    /*object builtins = import("builtins");
    object types = import("types");
    object module = types.attr("ModuleType");
    */

    // to_python_converter<DynamicComponent, dynamic_to_python, false>();

    // meta_any_from_python();

    py::class_<entt::type_info>(m, "TypeInfo")
        .def("name", &entt::type_info::name)
        .def("id", &entt::type_info::index)
        .def("hash", &entt::type_info::hash);

    py::module_ components = m.def_submodule("components", R"(Components module)");

    declare_python_component<groot::PlantGraph>(components, "PlantGraph");
    declare_python_component<PointNormals>(components, "PointNormals");
    declare_python_component<PointColors>(components, "PointColors");
    declare_python_component<PointCurvature>(components, "PointCurvature");
    declare_python_component<Cylinders>(components, "Cylinders");

    py::class_<PointCloud>(m, "PointCloud", py::buffer_protocol())
        //.def_property_readonly("type_id", entt::type_id<PointCloud>())
        .def_buffer([](PointCloud& c) {
            return create_buffer_info(c.cloud);
        });

    py::class_<PointNormals>(m, "PointNormals", py::buffer_protocol())
        //.def_property_readonly("type_id", entt::type_id<PointNormals>())
        .def_buffer([](PointNormals& n) {
            return create_buffer_info(n.normals);
        });

    py::class_<Cylinders>(m, "Cylinders")
        //.def_property_readonly("type_id", entt::type_id<Cylinders>())
        .def("__len__", [](const Cylinders& c) { return c.cylinders.size(); })
        .def(
            "__getitem__", [](Cylinders& c, size_t i) -> groot::CylinderWithPoints& { return c.cylinders.at(i); },
            py::return_value_policy::reference_internal);

    py::class_<groot::CylinderWithPoints>(m, "CylinderWithPoints", py::buffer_protocol())
        //.def_property_readonly("type_id", entt::type_id<groot::CylinderWithPoints>())
        .def_buffer([](groot::CylinderWithPoints& c) {
            return create_buffer_info(c.points);
        })
        .def_readwrite("cylinder", &groot::CylinderWithPoints::cylinder);

    py::class_<groot::Cylinder>(m, "Cylinder")
        //.def_property_readonly("type_id", entt::type_id<groot::Cylinder>())
        .def_readwrite("center", &groot::Cylinder::center)
        .def_readwrite("direction", &groot::Cylinder::direction)
        .def_readwrite("radius", &groot::Cylinder::radius)
        .def_readwrite("middle_height", &groot::Cylinder::middle_height);

    py::class_<groot::Point_3>(m, "Point3")
        .def_readwrite("x", (float groot::Point_3::*)&glm::vec3::x)
        .def_readwrite("y", (float groot::Point_3::*)&glm::vec3::y)
        .def_readwrite("z", (float groot::Point_3::*)&glm::vec3::z);

    py::class_<groot::Vector_3>(m, "Vector3")
        .def_readwrite("x", (float groot::Vector_3::*)&glm::vec3::x)
        .def_readwrite("y", (float groot::Vector_3::*)&glm::vec3::y)
        .def_readwrite("z", (float groot::Vector_3::*)&glm::vec3::z);

    create_plant_graph_component(m);
    create_task_module(m);
    create_entity_type(m);
    create_registry_type(m);

    py::module_ imgui = m.def_submodule("ImGui", "ImGui operations");
    create_imgui_module(imgui);
}

PYBIND11_MODULE(pygroot, m)
{
    create_pygroot_module(m);
}