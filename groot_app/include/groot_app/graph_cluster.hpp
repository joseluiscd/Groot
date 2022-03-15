#pragma once

#include <groot_app/command_gui.hpp>
#include <groot_app/components.hpp>
#include <groot_app/entt.hpp>
#include <groot_graph/plant_graph.hpp>
#include <list>
#include <vector>

GROOT_APP_API async::task<void> graph_cluster_fixed_interval_task(entt::handle h, size_t interval_count);

class GROOT_APP_LOCAL GraphClusterGui : public DialogGui {
public:
    GraphClusterGui(entt::handle h)
        : target(h)
    {}

    void draw_dialog() override;
    void schedule_commands(entt::registry& reg) override;
    std::string_view name() const override { return "Graph cluster"; }

    enum IntervalType : int {
        FixedIntervalDistance = 0,
        FixedIntervalCount,

        IntervalType_COUNT
    };

    enum CentroidType : int {
        CentroidMedian = 0,
        CentroidMean = 1,

        CentroidType_COUNT
    };

private:
    entt::handle target;
    static IntervalType selected_interval_type;
    static CentroidType selected_centroid_type;

    static float interval_distance;
    static int interval_count;

    static constexpr const char* centroid_type_labels[] = {
        "Median",
        "Mean",
    };
};
