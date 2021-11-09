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
struct bait::SystemTraits<ComputeNormals> {
    using Cmd = ComputeNormalsCmd;
    using Get = entt::get_t<const PointCloud>;
    using Result = PointNormals;

    static constexpr const std::string_view name = "Compute Normals";
};

struct ComputeNormals : public bait::GuiSystemImpl<bait::SystemImpl<bait::System<ComputeNormals>>> {
    static void draw_gui(Cmd& cmd);
    static PointNormals update_async(std::tuple<const PointCloud&, const Cmd&> data);
    static void update_sync(entt::handle h, PointNormals&& normals);
};

// Recenter cloud
// ------------------

struct RecenterCloudCmd {
};

template <>
struct bait::SystemTraits<RecenterCloud> {
    using Cmd = RecenterCloudCmd;
    using Get = entt::get_t<const PointCloud>;
    using Result = PointCloud;
};

struct RecenterCloud : public bait::SystemImpl<bait::System<RecenterCloud>> {
    static PointCloud update_async(std::tuple<const PointCloud&, const Cmd&> data);
    static void update_sync(entt::handle h, PointCloud&& new_cloud);
};

// Split cloud
// ----------------

struct SplitCloudCmd {
    float voxel_size = 1.0;
};

struct SplitCloudResult {
    std::vector<PointCloud> clouds;
    std::optional<std::vector<PointNormals>> normals;
    std::optional<std::vector<PointColors>> colors;
};

template <>
struct bait::SystemTraits<SplitCloud> {

};

struct SplitCloud : public bait::GuiSystemImpl<bait::SystemImpl<bait::System<SplitCloud>>> {
public:
    GuiState draw_gui() override;
    CommandState execute() override;
    void on_finish() override;

private:
    entt::registry& reg;
    PointCloud* cloud;
    std::optional<PointNormals*> normals;

    entt::entity target;

    groot::VoxelGrid grid;

    std::vector<PointCloud> result_clouds;
    std::vector<PointNormals> result_normals;

public:
    // Parameters
    float voxel_size = 1.0;

    // Result
    std::vector<entt::handle> result;
};

namespace cloud_view_system {

void init(entt::registry& reg);
void deinit(entt::registry& reg);
void run(entt::registry& reg);

}