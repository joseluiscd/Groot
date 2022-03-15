#pragma once

#include <groot_app/command_gui.hpp>
#include <groot_app/components.hpp>
#include <groot_app/entt.hpp>
#include <gfx/imgui/imgui.h>
#include <groot_graph/plant_graph.hpp>
#include <optional>
#include <string>
#include <spdlog/spdlog.h>

GROOT_APP_API async::task<void> geodesic_graph_command(entt::handle h);
GROOT_APP_API async::task<void> mst_graph_command(entt::handle h);

GROOT_APP_API async::task<void> graph_from_cloud_knn_task(entt::handle h, int k);
GROOT_APP_API async::task<void> graph_from_cloud_radius_task(entt::handle h, float radius);
GROOT_APP_API async::task<void> graph_from_cloud_alpha_shape_task(entt::handle h, float alpha = 0.0f, int components = 1);

struct GROOT_APP_LOCAL CreateGraph final {
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

    int selected_method = 0;
    int selected_root_find_method = 1;
    int selected_make_tree_method = 0;

    bool use_alpha_components = true;

    int k = 10;
    double radius = 1.0;
    float alpha = 0.0;
    int components = 1;

    groot::PlantGraph execute(PointCloud* cloud) const;
};

class GROOT_APP_API CreateGraphGui final : public DialogGui {
public:
    CreateGraphGui(entt::handle h)
        : cmd()
        , target(h)
    {
    }

    void schedule_commands(entt::registry& reg) override;
    void draw_dialog() override;
    std::string_view name() const override { return "Create Graph"; }

    static constexpr const char* root_find_labels[CreateGraph::kRootFindMethod_COUNT] = {
        "Z min",
        "Y min",
        "X min",
        "Z max",
        "Y max",
        "X max"
    };

    static constexpr const char* make_tree_method_labels[CreateGraph::kMakeTreeMethod_COUNT] = {
        "None",
        "Geodesic",
        "Minimum Spanning Tree",
    };
private:
    CreateGraph cmd;
    entt::handle target;
};
