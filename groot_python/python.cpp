#include "python.hpp"
#include "python_graph.hpp"
#include "python_imgui.hpp"
#include "python_task.hpp"

#include <groot_app/application.hpp>
#include <groot_app/cloud_io.hpp>
#include <groot_app/cloud_system.hpp>
#include <groot_app/components.hpp>
#include <groot_app/create_graph.hpp>
#include <groot_app/cylinder_connect.hpp>
#include <groot_app/cylinder_marching.hpp>
#include <groot_app/graph_cluster.hpp>
#include <groot_app/graph_io.hpp>
#include <groot_app/graph_resample.hpp>
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

        assert((bool)component);
        assert(component.type().info() == type);
        assert(component.data() != nullptr);

        return component;
    }

    void set_component_runtime(const entt::type_info& type, boost::python::object component)
    {
        auto& storage = e.registry()->storage(type);

        entt::meta_any constructed = entt::resolve(type).construct(component);
        assert((bool)constructed);

        if (storage->contains(e.entity())) {
            storage->erase(*e.registry(), e.entity());
        }

        storage->emplace(e.entity(), constructed.as_ref());
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

    std::string* get_name()
    {
        Name* name = e.try_get<Name>();
        return name == nullptr ? &name->name : nullptr;
    }

    void set_name(const std::string& v)
    {
        e.emplace_or_replace<Name>(v);
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
        {
            auto state = PyGILState_Ensure();
            std::invoke(init_func, boost::ref(*this));
            PyGILState_Release(state);
        }

        app.main_loop([this, &update_func](entt::registry&) {
            std::invoke(update_func, boost::ref(*this));
        });
    }

    void run_tasks()
    {
        sync_scheduler().run_all_tasks();
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

    class_<Registry, boost::noncopyable>("Registry",
        "The registry where all data is stored")
        .def_readonly("type_id", entt::type_id<Registry>())
        .def("selected", &Registry::selected)
        .def("entities", &Registry::entities)
        .def("load", &Registry::load)
        .def("save", &Registry::save)
        .def("load_ply", &Registry::load_ply)
        .def("load_graph", &Registry::load_graph)
        .def("new_entity", &Registry::new_entity)
        .def("schedule_task", &Registry::schedule_task)
        .def("run_tasks", &Registry::run_tasks)
        .def("run_viewer", &Registry::run_viewer,
            (arg("init_func") = builtins.attr("id"),
                arg("update_func") = builtins.attr("id")));

    class_<Entity>("Entity", "An entity in a registry", no_init)
        .def_readonly("type_id", entt::type_id<Entity>())
        .add_property("visible", &Entity::is_visible, &Entity::set_visible)
        .add_property("name", make_function(&Entity::get_name, return_internal_reference<1>()), &Entity::set_name)
        .def("destroy", &Entity::destroy)
        .def("remove_component", &Entity::remove_component_runtime)
        .def("move_component", &Entity::move_component)
        .def("__getitem__", &Entity::get_component_runtime, with_custodian_and_ward_postcall<0, 1>())
        .def("__setitem", &Entity::set_component_runtime)
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

    def(
        "compute_cardenas_et_al", +[](Entity e, float radius) {
            auto&& task = async::spawn(sync_scheduler(), [e]() {
                PointCloud* cloud = require_components<PointCloud>(e.e);
                return cloud;
            }).then(async_scheduler(), [radius](PointCloud* cloud) {
                  groot::PlantGraph graph = groot::from_cardenas_et_al(cloud->cloud.data(), cloud->cloud.size(), radius);
                  return graph;
              }).then(sync_scheduler(), [e](groot::PlantGraph&& graph) {
                e.e.emplace_or_replace<groot::PlantGraph>(std::move(graph));
            });

            return new PythonTask { std::move(task), "Cardenas et al." };
        },
        return_value_policy<manage_new_object>());

    def(
        "evaluate_difference", +[](Entity e, Entity f, bool create_entity) -> PythonTask* {
            ReleaseGilGuard guard;

            auto&& task = async::spawn(sync_scheduler(), [e, f]() {
                groot::PlantGraph* g1 = require_components<groot::PlantGraph>(e.e);
                groot::PlantGraph* g2 = require_components<groot::PlantGraph>(f.e);

                return std::make_pair(g1, g2);
            }).then(async_scheduler(), [](std::pair<groot::PlantGraph*, groot::PlantGraph*>&& graphs) {
                  groot::PlantGraph diff = groot::plant_graph_nn(*graphs.first, *graphs.second);
                  return std::make_tuple(diff, groot::plant_graph_nn_score(diff));
              }).then(sync_scheduler(), [create_entity, &reg = *e.e.registry()](std::tuple<groot::PlantGraph, float>&& v) {
                AcquireGilGuard guard;
                if (create_entity) {
                    entt::entity e = reg.create();
                    reg.emplace<groot::PlantGraph>(e, std::move(std::get<0>(v)));
                    return object(boost::python::make_tuple(Entity(reg, e), std::get<1>(v)));
                } else {
                    return object(v);
                }
            });

            return new PythonTask { std::move(task), "Evaluate difference. Returns value if create_entity == False. Returns (entity, value) if create_entity == True" };
        },
        (arg("entity1"), arg("entity2"), arg("create_entity") = false), return_value_policy<manage_new_object>());
}