#include "render.hpp"
#include <gfx/camera.hpp>

gfx::VertexArray::Layout point_layout = {
    { 0, 3, gfx::Type::Float } // Position
};

gfx::VertexArray::Layout direction_layout = {
    { 1, 3, gfx::Type::Float } // Direction
};

const char* point_vs = R"(
layout (location=0) in vec3 in_Position;

layout (location=kViewMatrix) uniform mat4 u_mvMatrix;
layout (location=kProjectionMatrix) uniform mat4 u_pMatrix;
layout (location=kPointSize) uniform float point_size;

void main()
{
    mat4 u_MvpMatrix = u_pMatrix * u_mvMatrix;
    gl_Position  = u_MvpMatrix * vec4(in_Position, 1.0);
    gl_PointSize = point_size;
};
)";

const char* point_fs = R"(
in  vec4 v_Color;
out vec4 out_FragColor;

layout (location=kColor) uniform vec3 u_color;

void main()
{
    out_FragColor = vec4(u_color, 1.0);
}
)";

const char* vector_vs = R"(
layout (location=0) in vec3 in_Position;
layout (location=1) in vec3 in_Direction;

out VertexData
{
    vec3 position;
    vec3 direction;
};

void main()
{
    position = in_Position;
    direction = in_Direction;
};
)";

const char* vector_gs = R"(
layout (points) in;
layout (line_strip, max_vertices=2) out;

layout (location = kProjectionMatrix) uniform mat4 mProj;
layout (location = kViewMatrix) uniform mat4 mView;

in VertexData
{
    vec3 position;
    vec3 direction;
} v_data[];

void main()
{
    vec3 p = v_data[0].position;
    vec3 n = p + normalize(v_data[0].direction);

    gl_Position = mProj * mView * vec4(p, 1.0);
    EmitVertex();

    gl_Position = mProj * mView * vec4(n, 1.0);
    EmitVertex();

    EndPrimitive();
}
)";

const char* vector_fs = R"(
out vec4 f_color;
 
layout (location=kColor) uniform vec3 u_color;

void main()
{
    f_color = vec4(u_color, 1.0);
}
)";
 
ShaderCollection::ShaderCollection()
    : shaders(
        { gfx::ShaderProgram::Builder("Points")
                .register_class<gfx::CameraLens>()
                .register_class<gfx::CameraRig>()
                .register_uniform<Color>()
                .register_uniform<PointSize>()
                .with_vertex_shader(point_vs)
                .with_fragment_shader(point_fs)
                .build_shared(),
            gfx::ShaderProgram::Builder("Vectors")
                .register_class<gfx::CameraLens>()
                .register_class<gfx::CameraRig>()
                .register_uniform<Color>()
                .with_vertex_shader(vector_vs)
                .with_geometry_shader(vector_gs)
                .with_fragment_shader(vector_fs)
                .build_shared() })
{
}

void ShaderCollection::init(entt::registry& reg)
{
    reg.set<ShaderCollection>();
}