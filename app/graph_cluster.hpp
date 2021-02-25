#pragma once

#include "application.hpp"
#include <groot/plant_graph.hpp>
#include <lemon/adaptors.h>
#include <list>
#include <vector>

class GraphCluster : public CommandGui {
public:
    GraphCluster(IDataSource<groot::PlantGraph>& _graph);

    CommandState execute () override;
    GuiState draw_gui() override;

private:
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

    IDataSource<groot::PlantGraph>& graph;
    IntervalType selected_interval_type = FixedIntervalCount;
    float interval_distance = 1.0;
    int interval_count = 50;

    CentroidType selected_centroid_type = CentroidMedian;

    static constexpr const char* centroid_type_labels[] = {
        "Median",
        "Mean",
    };
};