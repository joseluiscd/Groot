#pragma once

#include <groot_app/command_gui.hpp>
#include <groot_app/components.hpp>
#include <groot_app/entt.hpp>
#include <gfx/imgui/imgui.h>
#include <groot_graph/plant_graph.hpp>
#include <optional>
#include <string>
#include <spdlog/spdlog.h>

struct CreateGraph : public CommandGui {
    CreateGraph(entt::registry& registry)
        : CreateGraph(entt::handle(
            registry,
            registry.ctx<SelectedEntity>().selected))
    {
    }

    CreateGraph(entt::handle&& handle);

    enum Method {
        kRadius = 0,
        kKnn,
        kDelaunay,
        kAlphaShape,
        kCardenasDonosoEtAl,
        kMethod_COUNT
    };

    enum RootFindMethod {
        kMinZ = 0,
        kMinY,
        kMinX,
        kMaxZ,
        kMaxY,
        kMaxX,
        kRootFindMethod_COUNT
    };

    enum MakeTreeMethod {
        kNone = 0,
        kGeodesic,
        kMST,
        kMakeTreeMethod_COUNT,
    };

    entt::registry& registry;
    entt::entity target;
    PointCloud* cloud;

    int selected_method = 0;
    int selected_root_find_method = 1;
    int selected_make_tree_method = 0;

    bool use_alpha_components = true;

    int k = 10;
    double radius = 1.0;
    float alpha = 0.0;
    int components = 1;

    std::optional<groot::PlantGraph> result = {};

    GuiState draw_gui() override;
    CommandState execute() override;
    void on_finish(entt::registry& reg) override;

    static constexpr const char* root_find_labels[RootFindMethod::kRootFindMethod_COUNT] = {
        "Z min",
        "Y min",
        "X min",
        "Z max",
        "Y max",
        "X max"
    };

    static constexpr const char* make_tree_method_labels[MakeTreeMethod::kMakeTreeMethod_COUNT] = {
        "None",
        "Geodesic",
        "Minimum Spanning Tree",
    };
};

template <auto Operation>
struct SimpleGraphCommand : public CommandGui {
    SimpleGraphCommand(entt::registry& registry)
        : SimpleGraphCommand(entt::handle(
            registry,
            registry.ctx<SelectedEntity>().selected))
    {
    }

    SimpleGraphCommand(entt::handle&& handle);

    entt::registry& registry;
    entt::entity target;

    groot::PlantGraph* graph;
    std::optional<groot::PlantGraph> result;

    GuiState draw_gui() override
    {
        return GuiState::RunAsync;
    }

    CommandState execute() override
    {
        result = Operation(*graph);
        spdlog::info("Edges {}", boost::num_edges(*result));
        return CommandState::Ok;
    }

    void on_finish(entt::registry& reg) override
    {
        if (this->result) {
            registry.replace<groot::PlantGraph>(target, std::move(*this->result));
        }
    }
};

template <auto Operation>
SimpleGraphCommand<Operation>::SimpleGraphCommand(entt::handle&& handle)
    : registry(*handle.registry())
    , target(handle.entity())
{
    graph = require_components<groot::PlantGraph>(handle);
}

using GeodesicGraphCommand = SimpleGraphCommand<&groot::geodesic>;
using MSTGraphCommand = SimpleGraphCommand<&groot::minimum_spanning_tree>;