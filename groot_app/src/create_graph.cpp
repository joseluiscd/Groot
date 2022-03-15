#include <groot_app/components.hpp>
#include <groot_app/create_graph.hpp>
#include <spdlog/spdlog.h>

async::task<void> geodesic_graph_command(entt::handle h)
{
    return create_task()
        .require_component<groot::PlantGraph>(h)
        .then_async([](groot::PlantGraph* graph) {
            return groot::geodesic(*graph);
        })
        .emplace_component<groot::PlantGraph>(h);
}

async::task<void> mst_graph_command(entt::handle h)
{
    return create_task()
        .require_component<groot::PlantGraph>(h)
        .then_async([](groot::PlantGraph* graph) {
            return groot::minimum_spanning_tree(*graph);
        })
        .emplace_component<groot::PlantGraph>(h);
}

async::task<void> graph_from_cloud_knn_task(entt::handle h, int k)
{
    return create_task()
        .require_component<PointCloud>(h)
        .then_async([k](PointCloud* cloud) {
            return groot::from_search(cloud->cloud.data(), cloud->cloud.size(), groot::SearchParams { k, 0.0, groot::SearchType::kKnnSearch });
        })
        .emplace_component<groot::PlantGraph>(h);
}

async::task<void> graph_from_cloud_radius_task(entt::handle h, float radius)
{
    return create_task()
        .require_component<PointCloud>(h)
        .then_async([radius](PointCloud* cloud) {
            return groot::from_search(cloud->cloud.data(), cloud->cloud.size(), groot::SearchParams { 0, radius, groot::SearchType::kRadiusSearch });
        })
        .emplace_component<groot::PlantGraph>(h);
}

async::task<void> graph_from_cloud_alpha_shape_task(entt::handle h, float alpha, int components)
{
    return create_task()
        .require_component<PointCloud>(h)
        .then_async([alpha, components](PointCloud* cloud) {
            return groot::from_alpha_shape(cloud->cloud.data(), cloud->cloud.size(), alpha, components);
        })
        .emplace_component<groot::PlantGraph>(h);
}

void CreateGraphGui::schedule_commands(entt::registry& reg)
{
    reg.ctx<TaskManager>().push_task(
        "Creating graph",
        create_task()
            .then_async([_cmd = std::exchange(gui, nullptr)]() {
                std::unique_ptr<CreateGraph> cmd { _cmd };
                cmd->execute();
                return cmd;
            })
            .then_sync([&reg](std::unique_ptr<CreateGraph>&& cmd) {
                cmd->on_finish(reg);
            })
            .build());
}

CreateGraph::CreateGraph(entt::handle handle)
    : registry(*handle.registry())
    , target(handle.entity())
{
    cloud = require_components<PointCloud>(handle);
}

void CreateGraphGui::draw_dialog()
{
    ImGui::Text("Topology build method");
    ImGui::RadioButton("Radius search", (int*)&gui->selected_method, CreateGraph::Method::kRadius);
    ImGui::RadioButton("kNN", (int*)&gui->selected_method, CreateGraph::Method::kKnn);
    ImGui::RadioButton("3D Delaunay", (int*)&gui->selected_method, CreateGraph::Method::kDelaunay);
    ImGui::RadioButton("Alpha-shape", (int*)&gui->selected_method, CreateGraph::Method::kAlphaShape);
    ImGui::RadioButton("Cárdenas-Donoso et al. (2021)", (int*)&gui->selected_method, CreateGraph::Method::kCardenasDonosoEtAl);
    ImGui::Separator();

    ImGui::Text("Parameters");

    switch (gui->selected_method) {
    case CreateGraph::kRadius:
        ImGui::InputDouble("Radius", &gui->radius, 0.1, 0.5);
        break;

    case CreateGraph::kKnn:
        ImGui::InputInt("k", &gui->k, 1, 5);
        break;

    case CreateGraph::kDelaunay:
        ImGui::Text("--None--");
        break;
    case CreateGraph::kAlphaShape:
        ImGui::Checkbox("Desired components", &gui->use_alpha_components);

        if (gui->use_alpha_components) {
            ImGui::InputInt("# Components", &gui->components);
            gui->alpha = 0.0;
        } else {
            ImGui::InputFloat("Alpha", &gui->alpha);
        }
        break;
    case CreateGraph::kCardenasDonosoEtAl:
        ImGui::InputDouble("Point cloud resolution", &gui->radius);
        break;
    }

    ImGui::Separator();

    ImGui::Combo("Root find", &gui->selected_root_find_method, root_find_labels, CreateGraph::kRootFindMethod_COUNT);

    ImGui::Combo("Remove Loops", &gui->selected_make_tree_method, make_tree_method_labels, CreateGraph::kMakeTreeMethod_COUNT);
}

const groot::point_finder::PointFinder& choose_point_finder(int m)
{
    switch (m) {
    case CreateGraph::kMinX:
        return groot::point_finder::MinXPointFinder;
    case CreateGraph::kMinY:
        return groot::point_finder::MinYPointFinder;
    case CreateGraph::kMinZ:
        return groot::point_finder::MinZPointFinder;
    case CreateGraph::kMaxX:
        return groot::point_finder::MaxXPointFinder;
    case CreateGraph::kMaxY:
        return groot::point_finder::MaxYPointFinder;
    case CreateGraph::kMaxZ:
        return groot::point_finder::MaxZPointFinder;
    default:
        return groot::point_finder::MaxZPointFinder;
    }
}

void CreateGraph::execute()
{
    groot::PlantGraph graph;

    switch (selected_method) {
    case kKnn:
        spdlog::info("Running knn search...");
        graph = groot::from_search(cloud->cloud.data(), cloud->cloud.size(), groot::SearchParams { (int)k, (float)radius, groot::SearchType::kKnnSearch });
        break;
    case kRadius:
        spdlog::info("Running radius search...");
        graph = groot::from_search(cloud->cloud.data(), cloud->cloud.size(), groot::SearchParams { (int)k, (float)radius, groot::SearchType::kRadiusSearch });
        break;
    case kDelaunay:
        spdlog::info("Building 3D Delaunay...");
        graph = groot::from_delaunay(cloud->cloud.data(), cloud->cloud.size());
        break;
    case kAlphaShape:
        spdlog::info("Building Alpha shape...");
        graph = groot::from_alpha_shape(cloud->cloud.data(), cloud->cloud.size(), alpha, components);
        break;
    case kCardenasDonosoEtAl:
        spdlog::info("Building graph according to Cárdenas-Donoso et al. 2021");
        graph = groot::from_cardenas_et_al(cloud->cloud.data(), cloud->cloud.size(), radius, choose_point_finder(selected_root_find_method));
        break;
    default:
        throw std::runtime_error("Unknown method");
    }
    spdlog::info("Done");

    spdlog::info("Finding root point...");
    groot::find_root(graph, choose_point_finder(selected_root_find_method));
    spdlog::info("Found root point!");

    switch (selected_make_tree_method) {
    case kGeodesic:
        graph = groot::geodesic(graph);
        break;
    case kMST:
        graph = groot::minimum_spanning_tree(graph);
        break;
    default:
        break;
    }

    spdlog::info("Plant graph is created!");

    this->result = std::move(graph);
}

void CreateGraph::on_finish(entt::registry& reg)
{
    if (this->result) {
        registry.emplace_or_replace<groot::PlantGraph>(target, std::move(*this->result));
    }
}
