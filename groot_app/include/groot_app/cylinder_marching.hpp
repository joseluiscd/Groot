#pragma once

#include <groot_app/command_gui.hpp>
#include <groot_app/components.hpp>
#include <groot_app/entt.hpp>
#include <groot_graph/cylinder_marching.hpp>

struct CylinderFilterParams {
    bool radius = true;
    bool length = false;

    float radius_range[2] = { 0.0, 1.0 };
    float length_range[2] = { 0.0, 1.0 };

    static CylinderFilterParams only_radius(float min, float max)
    {
        CylinderFilterParams params;
        params.radius = true;
        params.length = false;

        params.radius_range[0] = min;
        params.radius_range[1] = max;

        return params;
    }

    static CylinderFilterParams only_length(float min, float max)
    {
        CylinderFilterParams params;
        params.radius = false;
        params.length = true;

        params.length_range[0] = min;
        params.length_range[1] = max;

        return params;
    }

    static CylinderFilterParams radius_and_length(float min_radius, float max_radius, float min_length, float max_lenght)
    {
        CylinderFilterParams params;
        params.radius = true;
        params.length = true;

        params.length_range[0] = min_length;
        params.length_range[1] = max_lenght;

        params.radius_range[0] = min_radius;
        params.radius_range[1] = max_radius;

        return params;
    }
};

GROOT_APP_API async::task<void> cylinder_marching_command(entt::handle h, const groot::Ransac::Parameters& params, float voxel_size = 1.0f);
GROOT_APP_API async::task<void> cylinder_filter_command(entt::handle h, const CylinderFilterParams& params);
GROOT_APP_API async::task<void> cylinder_point_filter_command(entt::handle h);

class GROOT_APP_LOCAL CylinderMarching : public DialogGui {
public:
    CylinderMarching(entt::handle handle)
        : target(handle)
    {
        require_components<PointCloud, PointNormals>(handle);
    }

    void draw_dialog() override;
    void schedule_commands(entt::registry& reg) override;
    std::string_view name() const override
    {
        return "Cylinder Marching";
    }

private:
    entt::handle target;

public:
    // Params
    int min_points = 20;
    float epsilon = 0.03f;
    float sampling = 0.06f;
    float normal_deviation = 25.0f;
    float overlook_probability = 0.01f;
    float voxel_size = 1.0f;
};

class GROOT_APP_LOCAL CylinderFilter : public DialogGui {
public:
    CylinderFilter(entt::handle handle)
        : target(handle)
    {
        require_components<Cylinders>(handle);
    }
    void draw_dialog() override;
    void schedule_commands(entt::registry& reg) override;

    std::string_view name() const override
    {
        return "Cylinder Filter";
    }
private:
    entt::handle target;

public:
    CylinderFilterParams params;
};

namespace cylinder_view_system {

GROOT_APP_LOCAL void init(entt::registry& reg);
GROOT_APP_LOCAL void deinit(entt::registry& reg);
GROOT_APP_LOCAL void run(entt::registry& reg);

}