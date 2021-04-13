#pragma once

#include <gfx/uniform.hpp>
#include <gfx/vertex_array.hpp>
#include <entt/entt.hpp>

using namespace entt::literals;

DEF_UNIFORM_SEMANTICS(Color, glm::vec3, "kColor");
DEF_UNIFORM_SEMANTICS(PointSize, float, "kPointSize");

extern gfx::VertexArray::Layout point_layout;
extern gfx::VertexArray::Layout direction_layout;

class ShaderCollection {
public:
    ShaderCollection();
    static void init(entt::registry& reg);

    enum ShaderID {
        Points = 0,
        Vectors,
        ShaderID_Count
    };

    std::shared_ptr<gfx::ShaderProgram> get_shader(ShaderID id) {
        return shaders[id];
    }

private:
    std::array<std::shared_ptr<gfx::ShaderProgram>, ShaderID_Count> shaders;
};


