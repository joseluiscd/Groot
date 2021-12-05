#include "python.hpp"
#include "entity.hpp"
#include "registry.hpp"
#include "python_graph.hpp"
#include "python_imgui.hpp"
#include "python_task.hpp"

#include "groot/cgal.hpp"
#include "groot/cloud.hpp"
#include <groot_app/open_workspace.hpp>
#include <groot_app/save_workspace.hpp>
#include <boost/core/noncopyable.hpp>
#include <boost/python/list.hpp>
#include <boost/python/numpy.hpp>
#include <boost/python/numpy/dtype.hpp>
#include <boost/python/numpy/ndarray.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/tuple.hpp>
#include <entt/meta/pointer.hpp>
#include <functional>
#include <groot_app/entt.hpp>
#include <groot_graph/plant_graph_compare.hpp>

using namespace entt::literals;

constexpr const entt::id_type convert_to_python = "convert_to_python"_hs;


struct meta_any_to_python {
    static PyObject* convert(const entt::meta_any& component)
    {
        spdlog::error("{}", component.type().info().name());
        if (!component.type()) {
            Py_RETURN_NONE;
        }

        assert((bool)component);
        assert(component.data() != nullptr);

        auto function = component.type().func(convert_to_python);
        assert((bool)function);
        assert(function.arity() == 0);
        assert(!function.is_static());
        assert(function.is_const());
        assert(function.ret() == entt::resolve<boost::python::object>());

        entt::meta_any result_any = function.invoke(component);
        assert(result_any);
        spdlog::error("{}", result_any.type().info().name());

        auto result = result_any.cast<boost::python::object>();

        return boost::python::incref(result.ptr());
    }
};

struct meta_any_from_python {
    meta_any_from_python()
    {
        boost::python::converter::registry::push_back(&convertible, &construct, boost::python::type_id<entt::meta_any>());
    }

    static void* convertible(PyObject* obj)
    {
        PyObject* type = PyObject_GetAttrString(obj, "type_id");
        if (type == nullptr) {
            return nullptr;
        }
        boost::python::extract<entt::type_info> info_extract(type);
        if (!info_extract.check()) {
            return nullptr;
        }
        return obj;
    }

    static void construct(PyObject* obj, boost::python::converter::rvalue_from_python_stage1_data* data)
    {
        void* storage = ((boost::python::converter::rvalue_from_python_storage<entt::meta_any>*)data)->storage.bytes;
        boost::python::object object { boost::python::handle<>(obj) };
        entt::type_info info = boost::python::extract<entt::type_info>(object.attr("type_id"));

        entt::meta_type type = entt::resolve(info);
        if (!type) {
            new (storage) entt::meta_any();
            return;
        }

        entt::meta_any constructed = type.construct(object);
        storage = new entt::meta_any(constructed.as_ref());
    }
};


boost::python::numpy::ndarray create_numpy_array(std::vector<groot::Point_3>& v)
{
    return boost::python::numpy::from_data(v.data(),
        boost::python::numpy::dtype::get_builtin<float>(),
        boost::python::make_tuple(v.size(), 3),
        boost::python::make_tuple(3 * sizeof(float), sizeof(float)),
        boost::python::object());
}

boost::python::numpy::ndarray create_numpy_array(std::vector<groot::Vector_3>& v)
{
    return boost::python::numpy::from_data(v.data(),
        boost::python::numpy::dtype::get_builtin<float>(),
        boost::python::make_tuple(v.size(), 3),
        boost::python::make_tuple(3 * sizeof(float), sizeof(float)),
        boost::python::object());
}

template <typename Object, typename T, size_t count>
boost::python::numpy::ndarray create_numpy_array(Object& v)
{
    return boost::python::numpy::from_data(&v,
        boost::python::numpy::dtype::get_builtin<T>(),
        boost::python::make_tuple(count),
        boost::python::make_tuple(sizeof(T)),
        boost::python::object());
}

template <typename Component>
boost::python::object convert_to_python_impl(const Component& c)
{
    return boost::python::object(boost::ref(c));
}

template <typename Component>
Component get_from_python_impl(boost::python::object o)
{
    return Component(std::move(boost::python::extract<Component&>(o)));
}

template <typename Component>
void declare_python_component(boost::python::scope& scope, const std::string_view& name)
{

    entt::meta<Component>()
        .type()
        .template ctor<get_from_python_impl<Component>>()
        .template func<convert_to_python_impl<Component>, entt::as_ref_t>(convert_to_python);

    scope.attr(name.data()) = entt::type_id<Component>();
}

BOOST_PYTHON_MODULE(pygroot)
{
    using namespace boost::python;
    namespace np = boost::python::numpy;

    np::initialize();
    object builtins = import("builtins");
    object types = import("types");
    object module = types.attr("ModuleType");

    to_python_converter<entt::meta_any, meta_any_to_python, false>();
    meta_any_from_python();

    class_<entt::type_info>("TypeInfo", no_init)
        .def("name", &entt::type_info::name)
        .def("id", &entt::type_info::index)
        .def("hash", &entt::type_info::hash);

    {
        object components_mod = module("components", "Namespace for all component types");
        scope().attr("components") = components_mod;
        scope components(components_mod);

        declare_python_component<groot::PlantGraph>(components, "PlantGraph");
        declare_python_component<PointNormals>(components, "PointNormals");
        declare_python_component<PointColors>(components, "PointColors");
        declare_python_component<PointCurvature>(components, "PointCurvature");
        declare_python_component<Cylinders>(components, "Cylinders");
    }

    class_<Name>("Name", no_init)
        .def_readonly("type_id", entt::type_id<Name>())
        .add_property(
            "name", +[](const Name& n) { return n.name; }, +[](Name& n, const std::string& newname) { n.name = newname; })
        .def(
            "__str__", +[](Name& n) { return n.name; });

    class_<PointCloud>("PointCloud", no_init)
        .def_readonly("type_id", entt::type_id<PointCloud>())
        .def(
            "points", +[](PointCloud& c) { return create_numpy_array(c.cloud); });

    class_<PointNormals>("PointNormals", no_init)
        .def_readonly("type_id", entt::type_id<PointNormals>())
        .def(
            "normals", +[](PointNormals& c) { return create_numpy_array(c.normals); });

    class_<Cylinders>("Cylinders", no_init)
        .def_readonly("type_id", entt::type_id<Cylinders>())
        .def(
            "__len__", +[](const Cylinders& c) { return c.cylinders.size(); })
        .def(
            "__getitem__", +[](Cylinders& c, size_t i) -> groot::CylinderWithPoints& { return c.cylinders.at(i); },
            return_internal_reference<1>());

    class_<groot::CylinderWithPoints>("CylinderWithPoints", no_init)
        .def_readonly("type_id", entt::type_id<groot::CylinderWithPoints>())
        .def(
            "points", +[](groot::CylinderWithPoints& c) {
                return create_numpy_array(c.points);
            })
        .def_readwrite("cylinder", &groot::CylinderWithPoints::cylinder);

    class_<groot::Cylinder>("Cylinder", no_init)
        .def_readonly("type_id", entt::type_id<groot::Cylinder>())
        .def_readwrite("center", &groot::Cylinder::center)
        .def_readwrite("direction", &groot::Cylinder::direction)
        .def_readwrite("radius", &groot::Cylinder::radius)
        .def_readwrite("middle_height", &groot::Cylinder::middle_height);

    class_<groot::Point_3>("Point_3", init<float, float, float>())
        .def_readwrite("x", (float groot::Point_3::*)&glm::vec3::x)
        .def_readwrite("y", (float groot::Point_3::*)&glm::vec3::y)
        .def_readwrite("z", (float groot::Point_3::*)&glm::vec3::z)
        .def("as_numpy", &create_numpy_array<groot::Point_3, float, 3>);

    class_<groot::Vector_3>("Vector_3", init<float, float, float>())
        .def_readwrite("x", (float groot::Vector_3::*)&glm::vec3::x)
        .def_readwrite("y", (float groot::Vector_3::*)&glm::vec3::y)
        .def_readwrite("z", (float groot::Vector_3::*)&glm::vec3::z)
        .def("as_numpy", &create_numpy_array<groot::Vector_3, float, 3>);

    create_plant_graph_component();
    create_imgui_module();
    create_task_module();
    create_entity_type();
    create_registry_type();

}