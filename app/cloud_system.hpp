#pragma once

#include "components.hpp"
#include "groot/cloud.hpp"
#include <bait/gui_system.hpp>
#include <groot/cgal.hpp>

// Forward declarations

struct ComputeNormals;
struct RecenterCloud;
struct SplitCloud;

// Compute normals
// -------------------------

struct ComputeNormalsCmd {
    int selected_k = 1;
    int k = 0;
    float radius = 1.0;
};


template <>
struct bait::SystemTraits<ComputeNormals> : bait::DefaultSystemTraits {
    using Cmd = ComputeNormalsCmd;
    using Get = entt::get_t<const PointCloud>;
    using Result = PointNormals;

    static constexpr const std::string_view name = "Compute Normals";
};

struct ComputeNormals : public bait::GuiSystemImpl<bait::SystemImpl<bait::System<ComputeNormals>>> {
    static void draw_gui(Cmd& cmd);
    static PointNormals update_async(Cmd&& cmd, const PointCloud& cloud);
    static void update_sync(entt::handle h, PointNormals&& normals);
};

// Recenter cloud
// ------------------

struct RecenterCloudCmd {
    enum : int {
        Centroid = 0,
        BoundCenter
    } mode = Centroid;
};

template <>
struct bait::SystemTraits<RecenterCloud> : bait::DefaultSystemTraits {
    using Cmd = RecenterCloudCmd;
    using Get = entt::get_t<const PointCloud>;
    using Result = PointCloud;

    static constexpr const std::string_view name = "Recenter cloud";
};

struct RecenterCloud : public bait::GuiSystemImpl<bait::SystemImpl<bait::System<RecenterCloud>>> {
    static PointCloud update_async(Cmd&& cmd, const PointCloud& cloud);
    static void update_sync(entt::handle h, PointCloud&& new_cloud);
    static void draw_gui(Cmd& cmd);
};

// Split cloud
// ----------------

struct SplitCloudCmd {
    float voxel_size = 1.0;
};

struct SplitCloudResult {
    std::vector<PointCloud> clouds;
    std::vector<PointNormals> normals;
    std::vector<PointColors> colors;
};

template <>
struct bait::SystemTraits<SplitCloud> : bait::DefaultSystemTraits {
    using Cmd = SplitCloudCmd;
    using Get = entt::get_t<const PointCloud>;
    using OptionalGet = entt::get_t<const PointNormals, const PointColors>;
    using Result = SplitCloudResult;

    static constexpr const std::string_view name = "Split Cloud in Voxels";
};

struct SplitCloud : public bait::GuiSystemImpl<bait::SystemImpl<bait::System<SplitCloud>>> {
    static void draw_gui(Cmd& cmd);
    static SplitCloudResult update_async(Cmd&& cmd, const PointCloud& cloud, const PointNormals* normals, const PointColors* colors);
    static void update_sync(entt::handle h, SplitCloudResult&& split);
};

// Cloud view system
// ------------------

struct CloudViewer : public bait::System<CloudViewer> {
    CloudViewer();
    ~CloudViewer();

    void init(entt::registry& reg);
    void clear(entt::registry& reg);
    void update(entt::registry& reg);

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
};

using CloudSystem = bait::SystemCollection<ComputeNormals, RecenterCloud, SplitCloud, CloudViewer>;

inline CloudSystem c() {
    return CloudSystem { };
}