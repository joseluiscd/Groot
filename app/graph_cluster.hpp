#pragma once

#include "command_gui.hpp"
#include "components.hpp"
#include <entt/entt.hpp>
#include <groot/plant_graph.hpp>
#include <list>
#include <vector>

class GraphCluster : public CommandGui {
public:
    GraphCluster(entt::handle&& handle);
    GraphCluster(entt::registry& _reg)
        : GraphCluster(entt::handle {
            _reg,
            _reg.ctx<SelectedEntity>().selected })
    {
    }

    CommandState execute() override;
    GuiState draw_gui() override;
    void on_finish() override;

private:
    entt::registry& reg;
    entt::entity target;

    groot::PlantGraph* graph;

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

    IntervalType selected_interval_type = FixedIntervalCount;

    CentroidType selected_centroid_type = CentroidMedian;

    static constexpr const char* centroid_type_labels[] = {
        "Median",
        "Mean",
    };

public:
    float interval_distance = 1.0;
    int interval_count = 50;

    groot::PlantGraph result;
};