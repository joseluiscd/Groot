#pragma once

#include <gfx/imgui/imfilebrowser.h>
#include <gfx/imgui/imgui.h>
#include <groot/plant_graph.hpp>
#include <string>
#include "application.hpp"
#include "data_output.hpp"
#include <entt/entt.hpp>
#include <optional>

struct CreateGraph : public CommandGui {
    CreateGraph(entt::registry& registry);

    enum Method {
        kRadius = 0,
        kKnn,
        kDelaunay,
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

    ImGui::FileBrowser open;
    std::string input_file = "";

    int selected_method = 0;
    int selected_root_find_method = 1;
    int selected_make_tree_method = 0;

    int k = 10;
    double radius = 1.0;


    std::optional<groot::PlantGraph> result = {};

    GuiState draw_gui() override;
    CommandState execute() override;
    void on_finish() override;

    std::variant<groot::PlantGraph, std::string> operation(void*) const;


    static constexpr const char* method_labels[3] = {
        "Radius search",
        "kNN",
        "3D Delaunay"
    };

    static constexpr const char* root_find_labels[] = {
        "Z min",
        "Y min",
        "X min",
        "Z max",
        "Y max",
        "X max"
    };

    static constexpr const char* make_tree_method_labels[] = {
        "None",
        "Geodesic",
        "Minimum Spanning Tree",
    };
};