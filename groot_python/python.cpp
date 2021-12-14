#include "python.hpp"
#include "entity.hpp"
#include "python_graph.hpp"
#include "python_imgui.hpp"
#include "python_task.hpp"
#include "registry.hpp"

#include "groot/cgal.hpp"
#include "groot/cloud.hpp"
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
#include <groot_app/open_workspace.hpp>
#include <groot_app/save_workspace.hpp>
#include <groot_graph/plant_graph_compare.hpp>

using namespace entt::literals;

constexpr const entt::id_type convert_to_python = "convert_to_python"_hs;
constexpr const entt::id_type convert_from_python = "convert_from_python"_hs;

#define RETURN_NONE_IF_NOT(EXPR) if(!static_cast<bool>(EXPR)) { Py_RETURN_NONE; }
#define RETURN_NULL_IF_NOT(EXPR) if(!static_cast<bool>(EXPR)) { return nullptr; }

struct dynamic_to_python {
    static PyObject* convert(const DynamicComponent& component)
    {
        auto function = entt::resolve(component.first).func(convert_to_python);
        RETURN_NONE_IF_NOT((bool)function);
        RETURN_NONE_IF_NOT(function.arity() == 1);
        RETURN_NONE_IF_NOT(function.is_static());
        RETURN_NONE_IF_NOT(function.arg(0) == entt::resolve<void*>());
        RETURN_NONE_IF_NOT(function.ret() == entt::resolve<boost::python::object>());

        entt::meta_any h(std::in_place_type<void*>, component.second);

        entt::meta_any result_any = function.invoke(entt::meta_handle(), h);
        RETURN_NONE_IF_NOT(result_any);
        auto result = result_any.cast<boost::python::object>();

        return boost::python::incref(result.ptr());
    }
};

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
boost::python::object convert_to_python_impl(void* c)
{
    return boost::python::object((Component*)c);
}

template <typename Component>
DynamicComponent get_from_python_impl(boost::python::object o)
{
    return DynamicComponent(entt::type_id<Component>(), (void*) boost::python::extract<Component*>(o));
}

template <typename Component>
void declare_python_component(boost::python::scope& scope, const std::string_view& name)
{

    entt::meta<Component>()
        .type()
        .template func<get_from_python_impl<Component>>(convert_from_python)
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

    to_python_converter<DynamicComponent, dynamic_to_python, false>();

    // meta_any_from_python();

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
