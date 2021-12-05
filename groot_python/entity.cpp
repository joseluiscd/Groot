#include "entity.hpp"

void create_entity_type()
{
    using namespace boost::python;

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

}