#include "create_graph.hpp"
#include "components.hpp"
#include <spdlog/spdlog.h>

CreateGraph::CreateGraph(entt::handle&& handle)
    : registry(*handle.registry())
    , target(handle.entity())
{
    cloud = require_components<PointCloud>(handle);
}

GuiState CreateGraph::draw_gui()
{
    bool show = true;
    ImGui::OpenPopup("Import PLY");
    if (ImGui::BeginPopupModal("Import PLY", &show, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {

        ImGui::Text("Topology build method");
        ImGui::RadioButton("Radius search", (int*)&selected_method, Method::kRadius);
        ImGui::RadioButton("kNN", (int*)&selected_method, Method::kKnn);
        ImGui::RadioButton("3D Delaunay", (int*)&selected_method, Method::kDelaunay);
        ImGui::RadioButton("Alpha-shape", (int*)&selected_method, Method::kAlphaShape);
        ImGui::RadioButton("Cárdenas-Donoso et al. (2021)", (int*)&selected_method, Method::kCardenasDonosoEtAl);
        ImGui::Separator();

        ImGui::Text("Parameters");

        switch (selected_method) {
        case kRadius:
            ImGui::InputDouble("Radius", &radius, 0.1, 0.5);
            break;

        case kKnn:
            ImGui::InputInt("k", &k, 1, 5);
            break;

        case kDelaunay:
            ImGui::Text("--None--");
            break;
        case kAlphaShape:
            ImGui::Checkbox("Desired components", &use_alpha_components);

            if (use_alpha_components) {
                ImGui::InputInt("# Components", &components);
                alpha = 0.0;
            } else {
                ImGui::InputFloat("Alpha", &alpha);
            }
            break;
        case kCardenasDonosoEtAl:
            ImGui::InputDouble("Point cloud resolution", &radius);
            break;
        }

        ImGui::Separator();

        ImGui::Combo("Root find", &selected_root_find_method, root_find_labels, kRootFindMethod_COUNT);

        ImGui::Combo("Remove Loops", &selected_make_tree_method, make_tree_method_labels, kMakeTreeMethod_COUNT);

        ImGui::Separator();

        if (ImGui::Button("Run")) {
            //All configured, run the command
            ImGui::EndPopup();
            return GuiState::RunAsync;
        }

        ImGui::EndPopup();
    }

    return show ? GuiState::Editing : GuiState::Close;
}

CommandState CreateGraph::execute()
{
    bool is_delaunay = false;
    bool is_alpha_shape = false;
    groot::SearchType search_val;

    groot::PlantGraph graph;
    
    switch (selected_method) {
    case kKnn:
        spdlog::info("Running knn search...");
        graph = groot::from_search(cloud->cloud.data(), cloud->cloud.size(), groot::SearchParams {
            .k = (int)k,
            .radius = (float)radius,
            .search = groot::SearchType::kKnnSearch
        });
        break;
    case kRadius:
        spdlog::info("Running radius search...");
        graph = groot::from_search(cloud->cloud.data(), cloud->cloud.size(), groot::SearchParams {
            .k = (int)k,
            .radius = (float)radius,
            .search = groot::SearchType::kRadiusSearch
        });
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
        graph = groot::from_cardenas_et_al(cloud->cloud.data(), cloud->cloud.size(), radius);
        break;
    default:
        error_string = "Unknown method";
        return CommandState::Error;
    }
    spdlog::info("Done");

    spdlog::info("Finding root point...");
    switch (selected_root_find_method) {
    case kMinX:
        groot::find_root(graph, groot::point_finder::MinX());
        break;
    case kMinY:
        groot::find_root(graph, groot::point_finder::MinY());
        break;
    case kMinZ:
        groot::find_root(graph, groot::point_finder::MinZ());
        break;
    case kMaxX:
        groot::find_root(graph, groot::point_finder::MaxX());
        break;
    case kMaxY:
        groot::find_root(graph, groot::point_finder::MaxY());
        break;
    case kMaxZ:
        groot::find_root(graph, groot::point_finder::MaxZ());
        break;
    default:
        error_string = "Unrecognized root find method";
        return CommandState::Error;
    }

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

    return CommandState::Ok;
}

void CreateGraph::on_finish()
{
    if (this->result) {
        registry.emplace_or_replace<groot::PlantGraph>(target, std::move(*this->result));
    }
}

