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
        .def(py::init<>())
        .def_readonly_static("type_id", &Registry::type_id)
        .def("selected", &Registry::selected)
        .def("entities", &Registry::entities)
        .def("load", &Registry::load)
        .def("save", &Registry::save)
        .def("load_ply", &Registry::load_ply)
        .def("load_graph", &Registry::load_graph)
        .def("new_entity", &Registry::new_entity)
        .def("schedule_task", &Registry::schedule_task)
        .def("create_viewer", &Registry::create_viewer, py::return_value_policy::take_ownership)
        .def("run_viewer", &Registry::run_viewer,
            arg("init_func") = py::none(),
            arg("update_func") = py::none());

    py::class_<AsyncViewer>(m, "Viewer", "Viewer")
        .def("step", &AsyncViewer::step)
        .def("should_close", &AsyncViewer::should_close);

    m.def(
        "compute_cardenas_et_al", +[](Entity e, float radius) {
            return PythonTask {
                async::spawn(sync_scheduler(), [e]() {
                    PointCloud* cloud = require_components<PointCloud>(e.e);
                    return cloud;
                }).then(async_scheduler(), [radius](PointCloud* cloud) {
                      groot::PlantGraph graph = groot::from_cardenas_et_al(cloud->cloud.data(), cloud->cloud.size(), radius);
                      return graph;
                  }).then(sync_scheduler(), [e](groot::PlantGraph&& graph) {
                    e.e.emplace_or_replace<groot::PlantGraph>(std::move(graph));
                })
            };
        });

    m.def(
        "compute_difference", +[](Entity e, Entity f) -> PythonTask {
            ReleaseGilGuard guard;
            return PythonTask {
                async::spawn(sync_scheduler(), [e, f]() {
                    groot::PlantGraph* g1 = require_components<groot::PlantGraph>(e.e);
                    groot::PlantGraph* g2 = require_components<groot::PlantGraph>(f.e);

                    return std::make_pair(g1, g2);
                }).then(async_scheduler(), [](std::pair<groot::PlantGraph*, groot::PlantGraph*>&& graphs) {
                      return groot::plant_graph_nn(*graphs.first, *graphs.second);
                  }).then(sync_scheduler(), [&reg = *e.e.registry()](groot::PlantGraph&& v) {
                    AcquireGilGuard guard;
                    entt::entity e = reg.create();
                    reg.emplace<groot::PlantGraph>(e, std::move(v));
                    return py::cast(Entity(reg, e));
                })
            };
        },
        arg("entity1"), arg("entity2"));
    m.def(
        "evaluate_difference_mse", [](Entity e) -> PythonTask {
            ReleaseGilGuard guard;
            return PythonTask {
                async::spawn(sync_scheduler(), [e]() {
                    return require_components<groot::PlantGraph>(e.e);
                }).then(async_scheduler(), [](groot::PlantGraph* g) {
                    py::gil_scoped_acquire guard;
                    return py::cast(groot::plant_graph_nn_score_mse(*g));
                })
            };
        });
}