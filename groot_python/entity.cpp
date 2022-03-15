#include "entity.hpp"
#include "python_task.hpp"

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
        .def("__getitem__", [](py::object self, const entt::type_info& type){
            Entity* e = py::cast<Entity*>(self);
            return e->get_component_runtime(self, type);
        })
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
        .def("split_cloud", [](Entity e, float voxel_size) {
                py::gil_scoped_release guard;

                return new PythonTask {
                    split_cloud_command(e.e, voxel_size)
                        .then(sync_scheduler(), [](std::vector<entt::handle>&& result){
                            py::gil_scoped_acquire guard;
                            py::list result_list;

                            for (auto i : result) {
                                result_list.append(Entity(i));
                            }
                            return (py::object) result_list;
                        })
                };
            },
            arg("voxel_size") = 1.0,
            py::return_value_policy::take_ownership)
        .def("rebuild_cloud_from_cylinders", &Entity::rebuild_cloud_from_cylinders)
        .def("build_graph_from_cylinders", &Entity::build_graph_from_cylinders)
        .def("graph_cluster_interval_count", &Entity::graph_cluster_interval_count)
        .def("graph_from_cloud_knn", &Entity::graph_from_cloud_knn)
        .def("graph_from_cloud_radius", &Entity::graph_from_cloud_radius, arg("radius"))
        .def("graph_from_alpha_shape", &Entity::graph_from_alpha_shape, arg("alpha"), arg("components") = 0)
        .def("graph_resample", &Entity::graph_resample)
        .def("match_graph", &Entity::match_graph)
        
        .def("save_ply", &Entity::save_ply)
        .def("save_graph", &Entity::save_graph)
        .def("repair_graph", &Entity::repair_graph)
        ;
}