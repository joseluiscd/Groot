#include "storage.hpp"

#include "application.hpp"
#include "cloud_io.hpp"
#include "cloud_system.hpp"
#include "components.hpp"
#include "create_graph.hpp"
#include "cylinder_connect.hpp"
#include "cylinder_marching.hpp"
#include "entt.hpp"
#include "graph_cluster.hpp"
#include "graph_io.hpp"
#include "graph_resample.hpp"
#include "groot/cgal.hpp"
#include "groot/cloud.hpp"
#include "open_workspace.hpp"
#include "python.hpp"
#include "python_imgui.hpp"
#include "python_task.hpp"
#include "python_graph.hpp"
#include "save_workspace.hpp"
#include <boost/core/noncopyable.hpp>
#include <boost/python/list.hpp>
#include <boost/python/numpy.hpp>
#include <boost/python/numpy/dtype.hpp>
#include <boost/python/numpy/ndarray.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/tuple.hpp>
#include <entt/meta/pointer.hpp>
#include <functional>

using namespace entt::literals;

constexpr const entt::id_type convert_to_python = "convert_to_python"_hs; 

template <typename Result>
Result run_task(async::task<Result>&& task)
{
    // Run all sync tasks in current thread until task finishes
    while (!task.ready()) {
        sync_scheduler().run_all_tasks();
    }

    return task.get();
}

struct meta_any_to_python {
    static PyObject* convert(const entt::meta_any& component)
    {
        spdlog::error("{}", component.type().info().name());
        if (!component.type()) {
            Py_RETURN_NONE;
        }

        assert((bool) component);
        assert(component.data() != nullptr);

        auto function = component.type().func(convert_to_python);
        assert((bool) function);
        assert(function.arity() == 0);
        assert(! function.is_static());
        assert(function.is_const());
        assert(function.ret() == entt::resolve<boost::python::object>());

        entt::meta_any result_any = function.invoke(component);
        assert(result_any);
        spdlog::error("{}", result_any.type().info().name());


        auto result = result_any.cast<boost::python::object>();

        return boost::python::incref(result.ptr());
    }
};

class Entity {
public:
    Entity(entt::registry& _reg, entt::entity _e)
        : e(_reg, _e)
    {
    }

    Entity(const entt::handle& h)
        : e(h)
    {
    }

    Entity(entt::handle&& h)
        : e(h)
    {
    }

    void destroy()
    {
        e.destroy();
    }

    template <typename Component>
    Component* get_component()
    {
        return e.try_get<Component>();
    }

    template <typename Component>
    void remove_component()
    {
        e.remove<Component>();
    }

    entt::meta_any get_component_runtime(const entt::type_info& type)
    {
        auto& storage = e.registry()->storage(type);
        if (!storage->contains(e)) {
            // No entity
            return entt::meta_any();
        }

        auto component = storage->get(e);

        assert((bool) component);
        assert(component.type().info() == type);
        assert(component.data() != nullptr);

        return component;
    }

    void remove_component_runtime(const entt::type_info& type)
    {
        auto& storage = e.registry()->storage(type);
        storage->erase(*e.registry(), e.entity());
    }

    void move_component(Entity& target, const entt::type_info& type)
    {
        ReleaseGilGuard guard;

        if (e.registry() != target.e.registry()) {
            throw std::runtime_error("Source and target entities must be in the same registry");
        }

        if (target.e == e) {
            // Nothing to do, as they are the same
            return;
        }

        auto& storage = e.registry()->storage(type);

        if (!storage->contains(e)) {
            throw std::runtime_error("Entity does not have this component");
        }

        storage->move_to(*e.registry(), e, target.e);
    }

    void select()
    {
        e.registry()->ctx<SelectedEntity>().selected = e.entity();
    }

    bool is_visible()
    {
        return e.all_of<Visible>();
    }

    void set_visible(bool v)
    {
        if (v) {
            e.emplace_or_replace<Visible>();
        } else {
            e.remove<Visible>();
        }
    }

    void compute_normals(
        int k,
        float radius)
    {
        ReleaseGilGuard guard;

        auto&& task = compute_normals_command(e, k, radius);
        run_task(std::move(task));
    }

    void cylinder_marching(
        int min_points,
        float epsilon,
        float sampling,
        float normal_deviation,
        float overlook_probability,
        float voxel_size)
    {
        ReleaseGilGuard guard;

        CylinderMarching cmd { entt::handle(e) };
        cmd.min_points = min_points;
        cmd.epsilon = epsilon;
        cmd.sampling = sampling;
        cmd.normal_deviation = normal_deviation;
        cmd.overlook_probability = overlook_probability;
        cmd.voxel_size = voxel_size;
        cmd.run(*e.registry());
    }

    void cylinder_filter(
        float radius_min,
        float radius_max,
        float length_min,
        float length_max)
    {
        ReleaseGilGuard guard;

        CylinderFilter cmd { entt::handle(e) };
        cmd.filter_radius = true;
        cmd.radius_range[0] = radius_min;
        cmd.radius_range[1] = radius_max;
        cmd.filter_length = true;
        cmd.length_range[0] = length_min;
        cmd.length_range[1] = length_max;
        cmd.run(*e.registry());
    }

    boost::python::list split_cloud(float voxel_size)
    {
        ReleaseGilGuard guard;

        SplitCloud cmd { entt::handle(e) };
        cmd.voxel_size = voxel_size;
        cmd.run(*e.registry());

        {
            AcquireGilGuard gil;

            boost::python::list result;
            for (auto i : cmd.result) {
                result.append(Entity(i));
            }
            return result;
        }
    }

    void rebuild_cloud_from_cylinders()
    {
        ReleaseGilGuard guard;

        CylinderPointFilter cmd { entt::handle(e) };
        cmd.run(*e.registry());
    }

    void build_graph_from_cylinders()
    {
        ReleaseGilGuard guard;

        CylinderConnection cmd { entt::handle(e) };
        cmd.run(*e.registry());
    }

    void graph_cluster(int intervals)
    {
        ReleaseGilGuard guard;

        GraphCluster cmd { entt::handle(e) };
        cmd.interval_count = intervals;
        cmd.run(*e.registry());
    }

    void graph_from_cloud_knn(int k)
    {
        ReleaseGilGuard guard;

        CreateGraph cmd { entt::handle(e) };
        cmd.selected_method = CreateGraph::Method::kKnn;
        cmd.k = k;
        cmd.run(*e.registry());
    }

    void graph_from_cloud_radius(float r)
    {
        ReleaseGilGuard guard;

        CreateGraph cmd { entt::handle(e) };
        cmd.selected_method = CreateGraph::Method::kRadius;
        cmd.radius = r;
        cmd.run(*e.registry());
    }

    void graph_from_alpha_shape(float k)
    {
        ReleaseGilGuard guard;

        CreateGraph cmd { entt::handle(e) };
        cmd.selected_method = CreateGraph::Method::kAlphaShape;
        cmd.alpha = k;
        cmd.run(*e.registry());
    }

    Entity graph_resample(float length)
    {
        ReleaseGilGuard guard;

        auto&& task = graph_resample_command(e, length);
        auto& sched = sync_scheduler();

        while (!task.ready()) {
            sched.run_all_tasks();
        }

        return Entity(*e.registry(), run_task(std::move(task)));
    }

    Entity match_graph(Entity other)
    {
        ReleaseGilGuard guard;

        auto&& task = graph_match_command(this->e, other.e);
        return Entity(*e.registry(), run_task(std::move(task)));
    }

private:
    entt::handle e;
};

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
        std::invoke(init_func, boost::ref(*this));
        app.main_loop([this, &update_func](entt::registry&) {
            std::invoke(update_func, boost::ref(*this));
        });
    }

    void schedule_task(PythonTask& t)
    {
        std::string name = boost::python::extract<std::string>(t.name);
        reg.ctx<TaskBroker>().push_task(name, std::move(t.ignore_result()));
    }

private:
    entt::registry reg;
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
void declare_python_component(boost::python::scope& scope, const std::string_view& name)
{

    entt::meta<Component>()
        .type()
        .template func<convert_to_python_impl<Component>, entt::as_ref_t>(convert_to_python);

    scope.attr(name.data()) = entt::type_id<Component>();
}

BOOST_PYTHON_MODULE(groot)
{
    using namespace boost::python;
    namespace np = boost::python::numpy;

    np::initialize();
    object builtins = import("builtins");
    object types = import("types");
    object module = types.attr("ModuleType");

    to_python_converter<entt::meta_any, meta_any_to_python, false>();

    class_<entt::type_info>("TypeInfo", no_init)
        .def("name", &entt::type_info::name)
        .def("id", &entt::type_info::index)
        .def("hash", &entt::type_info::hash);

    {
        object components_mod = module("components", "Namespace for all component types");
        scope().attr("components") = components_mod;
        scope components(components_mod);

        declare_python_component<Name>(components, "Name");
        declare_python_component<groot::PlantGraph>(components, "PlantGraph");
        declare_python_component<PointNormals>(components, "PointNormals");
        declare_python_component<PointColors>(components, "PointColors");
        declare_python_component<PointCurvature>(components, "PointCurvature");
        declare_python_component<Cylinders>(components, "Cylinders");
    }

    def(
        "get_imgui_context", +[]() {
            return (size_t)ImGui::GetCurrentContext();
        });

    class_<Registry, boost::noncopyable>("Registry",
        "The registry where all data is stored")
        .def("selected", &Registry::selected)
        .def("entities", &Registry::entities)
        .def("load", &Registry::load)
        .def("save", &Registry::save)
        .def("load_ply", &Registry::load_ply)
        .def("load_graph", &Registry::load_graph)
        .def("new_entity", &Registry::new_entity)
        .def("schedule_task", &Registry::schedule_task)
        .def("run_viewer", &Registry::run_viewer,
            (arg("init_func") = builtins.attr("id"),
                arg("update_func") = builtins.attr("id")));

    class_<Entity>("Entity", "An entity in a registry", no_init)
        .add_property("visible", &Entity::is_visible, &Entity::set_visible)
        .def("destroy", &Entity::destroy)
        .def("remove_component", &Entity::remove_component_runtime)
        .def("move_component", &Entity::move_component)
        .def("get_component", &Entity::get_component_runtime, with_custodian_and_ward_postcall<0, 1>())
        .def("select", &Entity::select)
        .def("compute_normals", &Entity::compute_normals,
            (arg("k") = 10, arg("radius") = 10.0f))
        .def("cylinder_marching", &Entity::cylinder_marching,
            (arg("min_points") = 20,
                arg("epsilon") = 0.03,
                arg("sampling") = 0.06,
                arg("normal_deviation") = 25.0,
                arg("overlook_probability") = 0.01,
                arg("voxel_size") = 1.0))
        .def("cylinder_filter", &Entity::cylinder_filter,
            (arg("radius_min") = 0.0,
                arg("radius_max") = 99.0,
                arg("length_min") = 0.0,
                arg("length_max") = 99.0))
        .def("split_cloud", &Entity::split_cloud)
        .def("rebuild_cloud_from_cylinders", &Entity::rebuild_cloud_from_cylinders)
        .def("build_graph_from_cylinders", &Entity::build_graph_from_cylinders)
        .def("graph_cluster", &Entity::graph_cluster)
        .def("graph_from_cloud_knn", &Entity::graph_from_cloud_knn)
        .def("graph_from_cloud_radius", &Entity::graph_from_cloud_radius)
        .def("graph_resample", &Entity::graph_resample)
        .def("match_graph", &Entity::match_graph);

    class_<Name>("Name", no_init)
        .add_property("name", +[](const Name& n) { return n.name; }, +[](Name& n, const std::string& newname) { n.name = newname; })
        .def("__str__", +[](Name& n) { return n.name; });

    class_<PointCloud>("PointCloud", no_init)
        .def(
            "points", +[](PointCloud& c) { return create_numpy_array(c.cloud); });

    class_<PointNormals>("PointNormals", no_init)
        .def(
            "normals", +[](PointNormals& c) { return create_numpy_array(c.normals); });

    class_<Cylinders>("Cylinders", no_init)
        .def(
            "__len__", +[](const Cylinders& c) { return c.cylinders.size(); })
        .def(
            "__getitem__", +[](Cylinders& c, size_t i) -> groot::CylinderWithPoints& { return c.cylinders.at(i); },
            return_internal_reference<1>());

    class_<groot::CylinderWithPoints>("CylinderWithPoints", no_init)
        .def(
            "points", +[](groot::CylinderWithPoints& c) {
                return create_numpy_array(c.points);
            })
        .def_readwrite("cylinder", &groot::CylinderWithPoints::cylinder);

    class_<groot::Cylinder>("Cylinder", no_init)
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
}