#pragma once

#include <groot/groot.hpp>
#include "gfx/imgui/imgui.h"
#include <groot_app/entt.hpp>
#include <gfx/camera.hpp>
#include <gfx/framebuffer.hpp>
#include <gfx/uniform.hpp>
#include <gfx/vertex_array.hpp>

using namespace entt::literals;

GFX_UNIFORM_SEMANTICS(Color, glm::vec3);
GFX_UNIFORM_SEMANTICS(PointSize, float);
GFX_UNIFORM_SEMANTICS(VectorSize, float);

using ColorUniform = gfx::Uniform<Color>;
using PointSizeUniform = gfx::Uniform<PointSize>;
using VectorSizeUniform = gfx::Uniform<VectorSize>;

enum Attribs {
    Position = 0,
    Direction,
    Radius,
    Height,
    ColorAttr,
};

constexpr const gfx::VertexArray::Layout point_layout = {
    gfx::attrib<glm::vec3>(Attribs::Position)
};

constexpr const gfx::VertexArray::Layout color_layout = {
    gfx::attrib<glm::vec3>(Attribs::ColorAttr)
};

constexpr const gfx::VertexArray::Layout point_color_layout = {
    gfx::attrib<glm::vec3>(Attribs::Position),
    gfx::attrib<glm::vec3>(Attribs::ColorAttr)
};

constexpr const gfx::VertexArray::Layout direction_layout = {
    gfx::attrib<glm::vec3>(Attribs::Direction)
};

constexpr const gfx::VertexArray::Layout cylinder_layout = {
    gfx::attrib<glm::vec3>(Attribs::Position),
    gfx::attrib<glm::vec3>(Attribs::Direction),
    gfx::attrib<float>(Attribs::Radius),
    gfx::attrib<float>(Attribs::Height)
};

struct GROOT_LOCAL RenderData {
    gfx::Framebuffer framebuffer;
    std::unique_ptr<gfx::CameraRig> camera;
    std::unique_ptr<gfx::PerspectiveCameraLens> lens;
    glm::ivec2 size;

    RenderData(const RenderData&) = delete;
    RenderData(RenderData&&) = default;
    RenderData& operator=(RenderData&&) = default;
    RenderData& operator=(const RenderData&) = delete;
};

struct GROOT_LOCAL TextRenderDrawList {
    std::string text_buffer;
    std::vector<glm::vec2> positions;
    std::vector<size_t> begin;

    void add_text(const glm::vec2& position, const char* text, size_t size);
    void dump_to_draw_list(ImDrawList* list, glm::vec2 offset = { 0, 0 }, glm::vec2 size = { 1, 1 });
};

class GROOT_LOCAL ShaderCollection {
public:
    ShaderCollection();
    static void init(entt::registry& reg);
    static void deinit(entt::registry& reg);

    enum ShaderID {
        Points = 0,
        PointsColor,
        Vectors,
        Cylinders,
        ShaderID_Count
    };

    std::shared_ptr<gfx::ShaderProgram> get_shader(ShaderID id)
    {
        return shaders[id];
    }

private:
    std::array<std::shared_ptr<gfx::ShaderProgram>, ShaderID_Count> shaders;
};
