#include "entity.hpp"

entt::type_info Entity::type_id = entt::type_id<Entity>();

void create_entity_type(py::module& m)
{
    using arg = py::arg;

    py::class_<Entity>(m, "Entity", "An entity in a registry")
        .def_readonly_static("type_id", &Entity::type_id)
        .def_property("visible", &Entity::is_visible, &Entity::set_visible)
        .def_property("name", &Entity::get_name, &Entity::set_name)
        .def("destroy", &Entity::destroy)
        .def("remove_component", &Entity::remove_component_runtime)
        .def("move_component", &Entity::move_component)
        .def("__getitem__", &Entity::get_component_runtime, py::return_value_policy::reference_internal)
        .def("__setitem__", &Entity::set_component_runtime)
        .def("select", &Entity::select)
        .def("compute_normals", &Entity::compute_normals,
            arg("k") = 10, arg("radius") = 10.0f)
        .def("cylinder_marching", &Entity::cylinder_marching,
            arg("min_points") = 20,
            arg("epsilon") = 0.03,
            arg("sampling") = 0.06,
            arg("normal_deviation") = 25.0,
            arg("overlook_probability") = 0.01,
            arg("voxel_size") = 1.0)
        .def("cylinder_filter", &Entity::cylinder_filter,
            arg("radius_min") = 0.0,
            arg("radius_max") = 99.0,
            arg("length_min") = 0.0,
            arg("length_max") = 99.0)
        .def("split_cloud", &Entity::split_cloud)
        .def("rebuild_cloud_from_cylinders", &Entity::rebuild_cloud_from_cylinders)
        .def("build_graph_from_cylinders", &Entity::build_graph_from_cylinders)
        .def("graph_cluster", &Entity::graph_cluster)
        .def("graph_from_cloud_knn", &Entity::graph_from_cloud_knn)
        .def("graph_from_cloud_radius", &Entity::graph_from_cloud_radius)
        .def("graph_resample", &Entity::graph_resample)
        .def("match_graph", &Entity::match_graph);
}