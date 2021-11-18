#include "python.hpp"
#include "components.hpp"
#include "cylinder_marching.hpp"
#include "cylinder_connect.hpp"
#include "entt/entity/fwd.hpp"
#include "groot/cgal.hpp"
#include "groot/cloud.hpp"
#include "import_ply.hpp"
#include "graph_cluster.hpp"
#include "open_workspace.hpp"
#include "save_workspace.hpp"
#include "create_graph.hpp"
#include "cloud_system.hpp"
#include <boost/core/noncopyable.hpp>
#include <boost/python/list.hpp>
#include <boost/python/numpy/dtype.hpp>
#include <boost/python/numpy/ndarray.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/tuple.hpp>
#include <entt/entt.hpp>
#include <functional>
#include "application.hpp"
#include <boost/python/numpy.hpp>

class Entity {
public:
    Entity(entt::registry& _reg, entt::entity _e)
        : e(_reg, _e)
    {}

    Entity(const entt::handle& h)
        : e(h)
    {}

    Entity(entt::handle&& h)
        : e(h)
    {}

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
            e.emplace<Visible>();
        } else {
            e.remove<Visible>();
        }
    }

    void compute_normals(
        int k ,
        float radius 
    )
    {
        ComputeNormals cmd{ entt::handle(e) };
        cmd.k = k; 
        cmd.radius = radius;

        cmd.run(*e.registry());
    }

    void cylinder_marching(
        int min_points,
        float epsilon,
        float sampling,
        float normal_deviation,
        float overlook_probability,
        float voxel_size
    )
    {
        CylinderMarching cmd{ entt::handle(e)};
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
        float length_max
    )
    {
        CylinderFilter cmd{entt::handle(e)};
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
        boost::python::list result;

        SplitCloud cmd{entt::handle(e)};
        cmd.voxel_size = voxel_size;
        cmd.run(*e.registry());

        for (auto i : cmd.result) {
            result.append(Entity(i));
        }
        return result;
    }

    void rebuild_cloud_from_cylinders()
    {
        CylinderPointFilter cmd{entt::handle(e)};
        cmd.run(*e.registry());
    }

    void build_graph_from_cylinders()
    {
        CylinderConnection cmd{entt::handle(e)};
        cmd.run(*e.registry());
    }
    
    void graph_cluster(int intervals)
    {
        GraphCluster cmd{entt::handle(e)};
        cmd.interval_count = intervals;
        cmd.run(*e.registry());
    }

    void graph_from_cloud_knn(int k)
    {
        CreateGraph cmd{entt::handle(e)};
        cmd.selected_method = CreateGraph::Method::kKnn;
        cmd.k = k;
        cmd.run(*e.registry());
    }

    void graph_from_alpha_shape(float k)
    {
        CreateGraph cmd{entt::handle(e)};
        cmd.selected_method = CreateGraph::Method::kAlphaShape;
        cmd.alpha = k;
        cmd.run(*e.registry());
    }

private:
    entt::handle e;
};

class Registry {
public:
    Registry()
        : reg()
    {
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
        OpenWorkspace cmd;
        cmd.set_file(filename);
        cmd.run(reg);
    }
    
    void save(const std::string& filename)
    {
        SaveWorkspace cmd{ reg };
        cmd.set_file(filename);
        cmd.run(reg);
    }

    Entity load_ply(const std::string& filename)
    {
        ImportPLY cmd(reg);
        cmd.input_file = filename;
        cmd.run(reg);
        return Entity(reg, cmd.result);
    }

    Entity new_entity()
    {
        return Entity(reg, reg.create());
    }

    void run_viewer()
    {
        Application app(reg);
        app.main_loop();
    }

private:
    entt::registry reg;
};


boost::python::numpy::ndarray create_numpy_array(std::vector<groot::Point_3>& v)
{
    return boost::python::numpy::from_data(v.data(),
                boost::python::numpy::dtype::get_builtin<float>(),
                boost::python::make_tuple(v.size(), 3),
                boost::python::make_tuple(3*sizeof(float), sizeof(float)),
                boost::python::object());
}

boost::python::numpy::ndarray create_numpy_array(std::vector<groot::Vector_3>& v)
{
    return boost::python::numpy::from_data(v.data(),
                boost::python::numpy::dtype::get_builtin<float>(),
                boost::python::make_tuple(v.size(), 3),
                boost::python::make_tuple(3*sizeof(float), sizeof(float)),
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

BOOST_PYTHON_MODULE(groot)
{
    using namespace boost::python;
    namespace np = boost::python::numpy;

    np::initialize();


    class_<Registry, boost::noncopyable>("Registry",
        "The registry where all data is stored")
        .def("selected", &Registry::selected) 
        .def("entities", &Registry::entities)
        .def("load", &Registry::load)
        .def("save", &Registry::save)
        .def("load_ply", &Registry::load_ply)
        .def("new_entity", &Registry::new_entity)
        .def("run_viewer", &Registry::run_viewer)
        ;

    class_<Entity>("Entity", "An entity in a registry", no_init)
        .add_property("visible", &Entity::is_visible, &Entity::set_visible)
        .def("point_cloud", &Entity::get_component<PointCloud>, return_internal_reference<1>())
        .def("point_normals", &Entity::get_component<PointNormals>, return_internal_reference<1>())
        .def("cylinders", &Entity::get_component<Cylinders>, return_internal_reference<1>())
        .def("graph", &Entity::get_component<groot::PlantGraph>, return_internal_reference<1>())
        .def("select", &Entity::select)
        .def("compute_normals", &Entity::compute_normals,
            (arg("k")=10, arg("radius")=10.0f))
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
    ;

    class_<PointCloud>("PointCloud", no_init)
        .def("points", +[](PointCloud& c){ return create_numpy_array(c.cloud);})
    ;

    class_<PointNormals>("PointNormals", no_init)
        .def("normals", +[](PointNormals& c){ return create_numpy_array(c.normals);})
    ;

    class_<Cylinders>("Cylinders", no_init)
        .def("__len__", +[](const Cylinders& c){ return c.cylinders.size(); })
        .def("__getitem__", +[](Cylinders& c, size_t i) -> groot::CylinderWithPoints& { return c.cylinders.at(i); },
            return_internal_reference<1>())
    ;

    class_<groot::CylinderWithPoints>("CylinderWithPoints", no_init)
        .def("points", +[](groot::CylinderWithPoints& c) 
        {
            return create_numpy_array(c.points);
        })
        .def_readwrite("cylinder", &groot::CylinderWithPoints::cylinder)
    ;

    class_<groot::Cylinder>("Cylinder", no_init)
        .def_readwrite("center", &groot::Cylinder::center)
        .def_readwrite("direction", &groot::Cylinder::direction)
        .def_readwrite("radius", &groot::Cylinder::radius)
        .def_readwrite("middle_height", &groot::Cylinder::middle_height)
        ;
    
    class_<groot::Point_3>("Point_3", init<float, float, float>())
        .def_readwrite("x", (float groot::Point_3::*) &glm::vec3::x)
        .def_readwrite("y", (float groot::Point_3::*) &glm::vec3::y)
        .def_readwrite("z", (float groot::Point_3::*) &glm::vec3::z)
        .def("as_numpy", &create_numpy_array<groot::Point_3, float, 3>)
    ;

    class_<groot::Vector_3>("Vector_3", init<float, float, float>())
        .def_readwrite("x", (float groot::Vector_3::*) &glm::vec3::x)
        .def_readwrite("y", (float groot::Vector_3::*) &glm::vec3::y)
        .def_readwrite("z", (float groot::Vector_3::*) &glm::vec3::z)
        .def("as_numpy", &create_numpy_array<groot::Vector_3, float, 3>)
        ;
    
}