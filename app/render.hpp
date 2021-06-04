#pragma once

#include <gfx/uniform.hpp>
#include <gfx/vertex_array.hpp>
#include <entt/entt.hpp>
#include <gfx/framebuffer.hpp>
#include <gfx/camera.hpp>

using namespace entt::literals;

DEF_UNIFORM_SEMANTICS(Color, glm::vec3, "kColor");
DEF_UNIFORM_SEMANTICS(PointSize, float, "kPointSize");

extern gfx::VertexArray::Layout point_layout;
extern gfx::VertexArray::Layout direction_layout;
extern gfx::VertexArray::Layout cylinder_layout;

enum Attribs {
    Position = 0,
    Direction,
    Radius,
    Height
};

struct RenderData {
    gfx::Framebuffer framebuffer;
    std::unique_ptr<gfx::CameraRig> camera;
    std::unique_ptr<gfx::PerspectiveCameraLens> lens;
    glm::ivec2 size;

    RenderData(const RenderData&) = delete;
    RenderData(RenderData&&) = default;
    RenderData& operator=(RenderData&&) = default;
    RenderData& operator=(const RenderData&) = delete;
};

class ShaderCollection {
public:
    ShaderCollection();
    static void init(entt::registry& reg);
    static void deinit(entt::registry& reg);

    enum ShaderID {
        Points = 0,
        Vectors,
        Cylinders,
        ShaderID_Count
    };

    std::shared_ptr<gfx::ShaderProgram> get_shader(ShaderID id) {
        return shaders[id];
    }

private:
    std::array<std::shared_ptr<gfx::ShaderProgram>, ShaderID_Count> shaders;
};


