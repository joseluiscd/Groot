#include <groot_app/graph_cluster.hpp>
#include <groot_app/components.hpp>
#include <groot_graph/plant_graph.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <gfx/imgui/imgui.h>
#include <groot/cgal.hpp>
#include <list>
#include <spdlog/spdlog.h>
#include <vector>

GraphCluster::GraphCluster(entt::handle&& handle)
    : reg(*handle.registry())
{
    target = handle.entity();
    if (reg.valid(target) && reg.all_of<groot::PlantGraph>(target)) {
        graph = &reg.get<groot::PlantGraph>(target);
    } else {
        throw std::runtime_error("Selected entity must have PlantGraph component");
    }
}

// Map of (vertex_index) -> (cluster ID, component ID)
using ClusterMap = boost::iterator_property_map<
    std::vector<std::pair<size_t, size_t>>::iterator,
    boost::property_map<groot::PlantGraph, boost::vertex_index_t>::type>;

struct IntervalFilterOperator {
    IntervalFilterOperator() {};
    IntervalFilterOperator(ClusterMap _map, size_t _interval)
        : map(_map)
        , interval(_interval)
    {
    }

    bool operator()(const groot::Vertex& e) const { return map[e].first == interval; }

    ClusterMap map;
    size_t interval;
};

// TODO: Move to groot_graph library
CommandState GraphCluster::execute()
{
    groot::PropertyMap<float> distance_map;
    float max_root_distance;

    groot::PlantGraph g = groot::geodesic(*graph, &distance_map, &max_root_distance);

    std::vector<std::pair<size_t, size_t>> clusters(boost::num_vertices(g));

    // Points of base graph -> cluster (depth, id)
    ClusterMap clusters_map = ClusterMap(clusters.begin(), boost::get(boost::vertex_index, g));

    // Compute the interval of each vertex
    auto [it, end] = boost::vertices(g);
    for (; it != end; ++it) {
        float dist = distance_map[*it];
        size_t interval = float(interval_count) * dist / max_root_distance;
        if (interval == interval_count)
            --interval;

        clusters_map[*it].first = interval;
    }

    // Find the connected components in each interval (in original graph)
    for (size_t interval = 0; interval < interval_count; interval++) {
        boost::filtered_graph<groot::PlantGraph, boost::keep_all, IntervalFilterOperator> filtered(
            *graph,
            boost::keep_all(),
            IntervalFilterOperator(clusters_map, interval));

        auto components = boost::make_transform_value_property_map(
            [&](ClusterMap::reference r) -> size_t& {
                return r.second;
            },
            clusters_map);

        boost::connected_components(filtered, components);
    }

    // Create the simplified graph
    groot::PlantGraph simplified;
    auto vertex_indices = boost::get(boost::vertex_index, simplified);
    size_t next_index = 0;
    std::map<std::pair<size_t, size_t>, groot::Vertex> cluster_vertices;

    // Map (simpified vertex) -> (list of points in the cluster)
    std::vector<std::vector<groot::cgal::Point_3>> cluster_points(boost::num_vertices(g));
    auto cluster_points_map = boost::make_iterator_property_map(cluster_points.begin(), boost::get(boost::vertex_index, simplified));

    auto [v_it, v_end] = boost::vertices(g);
    for (; v_it != v_end; ++v_it) {
        auto [cluster_origin, inserted] = cluster_vertices.insert({ clusters_map[*v_it], groot::PlantGraph::null_vertex() });
        if (inserted) {
            auto new_vertex = boost::add_vertex(simplified);
            cluster_origin->second = new_vertex;
        }

        cluster_points_map[cluster_origin->second].push_back(g[*v_it].position);

        auto [e_it, e_end] = boost::out_edges(*v_it, g);
        for (; e_it != e_end; ++e_it) {
            auto origin = boost::source(*e_it, g);
            auto target = boost::target(*e_it, g);

            auto [cluster_target, inserted] = cluster_vertices.insert({ clusters_map[target], groot::PlantGraph::null_vertex() });
            if (inserted) {
                auto new_vertex = boost::add_vertex(simplified);
                cluster_target->second = new_vertex;
            }

            if (cluster_origin != cluster_target) {
                boost::add_edge(cluster_origin->second, cluster_target->second, simplified);
            }
        }
    }

    auto root_cluster = clusters_map[g.m_property->root_index];
    simplified.m_property->root_index = cluster_vertices[root_cluster];

    points.points.resize(boost::num_vertices(simplified));
    auto points_map = groot::make_vertex_property_map(points.points, simplified);

    auto [s_it, s_end] = boost::vertices(simplified);
    for (; s_it != s_end; ++s_it) {
        std::vector<groot::cgal::Point_3>& point_list = cluster_points_map[*s_it];

        groot::cgal::Vector_3 center(0, 0, 0);
        for (auto it = point_list.begin(); it != point_list.end(); ++it) {
            center = center + groot::cgal::Vector_3(it->x(), it->y(), it->z());
        }

        center /= point_list.size();
        simplified[*s_it].position = groot::cgal::Point_3(center.x(), center.y(), center.z());
    }
    points.points = std::move(cluster_points);
    points.points.shrink_to_fit();

    groot::reindex_edges(simplified);
    groot::recompute_edge_lengths(simplified);
    result = std::move(simplified);
    return CommandState::Ok;
}

GuiState GraphCluster::draw_gui()
{
    bool show = true;
    if (ImGui::BeginPopupModal("Graph Clustering", &show)) {
        ImGui::RadioButton("Fixed interval distance", (int*)&selected_interval_type, FixedIntervalDistance);
        ImGui::RadioButton("Fixed interval count", (int*)&selected_interval_type, FixedIntervalCount);

        ImGui::Separator();

        switch (selected_interval_type) {
        case FixedIntervalDistance:
            ImGui::InputFloat("Distance", &interval_distance);
            break;

        case FixedIntervalCount:
            ImGui::InputInt("Interval count", &interval_count);
            break;

        default:
            break;
        }

        ImGui::Separator();

        ImGui::Combo("Centroid selection", (int*)&selected_centroid_type, centroid_type_labels, CentroidType_COUNT);

        ImGui::Separator();

        if (ImGui::Button("Run")) {
            ImGui::EndPopup();
            return GuiState::RunAsync;
        }

        ImGui::EndPopup();
    }

    ImGui::OpenPopup("Graph Clustering");

    return show ? GuiState::Editing : GuiState::Close;
}

void GraphCluster::on_finish(entt::registry& reg)
{
    reg.emplace_or_replace<groot::PlantGraph>(target, std::move(result));
    reg.emplace_or_replace<PlantGraphNodePoints>(target, std::move(points));
}
