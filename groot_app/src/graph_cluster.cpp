#include <boost/graph/connected_components.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <gfx/imgui/imgui.h>
#include <groot/cgal.hpp>
#include <groot_app/components.hpp>
#include <groot_app/graph_cluster.hpp>
#include <groot_graph/cluster.hpp>
#include <groot_graph/line_graph.hpp>
#include <groot_graph/plant_graph.hpp>
#include <list>
#include <spdlog/spdlog.h>
#include <vector>

async::task<void> graph_cluster_fixed_interval_task(entt::handle h, size_t interval_count)
{
    return create_task()
        .require_component<groot::PlantGraph>(h)
        .then_async([interval_count](groot::PlantGraph* graph) {
            PlantGraphNodePoints points;
            groot::PlantGraph simplified = graph_cluster_fixed_groups(*graph, interval_count, &points.points);

            return std::make_tuple(
                std::move(points),
                std::move(simplified));
        })
        .emplace_components<PlantGraphNodePoints, groot::PlantGraph>(h);
}

async::task<void> graph_line_transform_task(entt::handle h)
{
    return create_task()
        .require_component<groot::PlantGraph>(h)
        .then_async([](groot::PlantGraph* graph) {
            return groot::compute_line_graph(*graph);
        })
        .emplace_component<groot::PlantGraph>(h);
}

GraphClusterGui::IntervalType GraphClusterGui::selected_interval_type = GraphClusterGui::FixedIntervalCount;
GraphClusterGui::CentroidType GraphClusterGui::selected_centroid_type = GraphClusterGui::CentroidMean;

float GraphClusterGui::interval_distance = 10.0f;
int GraphClusterGui::interval_count = 30;

void GraphClusterGui::draw_dialog()
{
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
}

void GraphClusterGui::schedule_commands(entt::registry& reg)
{
    if (selected_interval_type == FixedIntervalCount) {
        reg.ctx<TaskManager>()
            .push_task("Graph Clusterin", graph_cluster_fixed_interval_task(target, interval_count));
    }
}
