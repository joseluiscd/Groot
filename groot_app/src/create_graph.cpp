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
            return groot::from_search_knn(cloud->cloud.data(), cloud->cloud.size(), k);
        })
        .emplace_component<groot::PlantGraph>(h);
}

async::task<void> graph_from_cloud_radius_task(entt::handle h, float radius)
{
    return create_task()
        .require_component<PointCloud>(h)
        .then_async([radius](PointCloud* cloud) {
            return groot::from_search_radius(cloud->cloud.data(), cloud->cloud.size(), radius);
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
            .require_component<PointCloud>(target)
            .then_async([cmd = this->cmd](PointCloud* cloud) {
                return cmd.execute(cloud);
            })
            .emplace_component<groot::PlantGraph>(target)
            .build());
}

void CreateGraphGui::draw_dialog()
{
    ImGui::Text("Topology build method");
    ImGui::RadioButton("Radius search", (int*)&cmd.selected_method, CreateGraph::Method::kRadius);
    ImGui::RadioButton("kNN", (int*)&cmd.selected_method, CreateGraph::Method::kKnn);
    ImGui::RadioButton("3D Delaunay", (int*)&cmd.selected_method, CreateGraph::Method::kDelaunay);
    ImGui::RadioButton("Alpha-shape", (int*)&cmd.selected_method, CreateGraph::Method::kAlphaShape);
    ImGui::RadioButton("Cárdenas-Donoso et al. (2021)", (int*)&cmd.selected_method, CreateGraph::Method::kCardenasDonosoEtAl);
    ImGui::Separator();

    ImGui::Text("Parameters");

    switch (cmd.selected_method) {
    case CreateGraph::kRadius:
        ImGui::InputDouble("Radius", &cmd.radius, 0.1, 0.5);
        break;

    case CreateGraph::kKnn:
        ImGui::InputInt("k", &cmd.k, 1, 5);
        break;

    case CreateGraph::kDelaunay:
        ImGui::Text("--None--");
        break;
    case CreateGraph::kAlphaShape:
        ImGui::Checkbox("Desired components", &cmd.use_alpha_components);

        if (cmd.use_alpha_components) {
            ImGui::InputInt("# Components", &cmd.components);
            cmd.alpha = 0.0;
        } else {
            ImGui::InputFloat("Alpha", &cmd.alpha);
        }
        break;
    case CreateGraph::kCardenasDonosoEtAl:
        ImGui::InputDouble("Point cloud resolution", &cmd.radius);
        break;
    }

    ImGui::Separator();

    ImGui::Combo("Root find", &cmd.selected_root_find_method, root_find_labels, CreateGraph::kRootFindMethod_COUNT);

    ImGui::Combo("Remove Loops", &cmd.selected_make_tree_method, make_tree_method_labels, CreateGraph::kMakeTreeMethod_COUNT);
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

groot::PlantGraph CreateGraph::execute(PointCloud* cloud) const
{
    groot::PlantGraph graph;

    switch (selected_method) {
    case kKnn:
        spdlog::info("Running knn search...");
        graph = groot::from_search_knn(cloud->cloud.data(), cloud->cloud.size(), k);
        break;
    case kRadius:
        spdlog::info("Running radius search...");
        graph = groot::from_search_radius(cloud->cloud.data(), cloud->cloud.size(), radius);
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

    return graph;
}
