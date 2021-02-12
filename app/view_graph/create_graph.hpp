#pragma once

#include "imfilebrowser.h"
#include <gfx/imgui/imgui.h>
#include <groot/skeleton.hpp>
#include <string>
#include "operation.hpp"

struct CreateGraph : public Operation<groot::PlantGraph> {
    CreateGraph(std::function<void(groot::PlantGraph&&)> on_result);

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

    ImGui::FileBrowser open;
    ImGui::FileBrowser save;

    std::string input_file = "";
    std::string output_file = "";

    int selected_method = 0;
    int selected_root_find_method = 0;
    int selected_make_tree_method = 0;

    int k = 0;
    double radius = 1.0;

    void window();
    std::variant<groot::PlantGraph, std::string> operation() const;


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