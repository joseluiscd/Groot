#include "registry.hpp"
#include <groot_app/entt.hpp>
#include <groot_graph/plant_graph_compare.hpp>

entt::type_info Registry::type_id = entt::type_id<Registry>();

void create_registry_type(py::module& m)
{
    using arg = py::arg;
    py::module_ builtins = py::module_::import("builtins");

    py::class_<Registry>(m, "Registry",
        "The registry where all data is stored")
        .def_readonly_static("type_id", &Registry::type_id)
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
            arg("init_func") = py::none(),
            arg("update_func") = py::none());

    m.def(
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
        py::return_value_policy::take_ownership);

    m.def(
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
                    return py::object(py::make_tuple(Entity(reg, e), std::get<1>(v)));
                } else {
                    return py::cast(v);
                }
            });

            return new PythonTask { std::move(task), "Evaluate difference. Returns value if create_entity == False. Returns (entity, value) if create_entity == True" };
        },
        arg("entity1"), arg("entity2"), arg("create_entity") = false,
        py::return_value_policy::take_ownership);
}