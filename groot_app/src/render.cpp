#include <groot_app/render.hpp>
#include <gfx/camera.hpp>



const char* point_vs = R"(
layout (location=0) in vec3 in_Position;

uniform mat4 u_mvMatrix;
uniform mat4 u_pMatrix;
uniform float point_size;

void main()
{
    mat4 u_MvpMatrix = u_pMatrix * u_mvMatrix;
    gl_Position  = u_MvpMatrix * vec4(in_Position, 1.0);
    gl_PointSize = point_size;
};
)";

const char* point_fs = R"(
out vec4 out_FragColor;

uniform vec3 u_color;

void main()
{
    vec2 pp = gl_PointCoord - vec2(0.5, 0.5);
    if (dot(pp, pp) > 0.25) discard;
    out_FragColor = vec4(u_color, 1.0);
}
)";

const char* point_color_vs = R"(
layout (location=0) in vec3 in_Position;
layout (location=4) in vec3 in_Color;

uniform mat4 u_mvMatrix;
uniform mat4 u_pMatrix;
uniform float point_size;

out vec4 v_Color;

void main()
{
    mat4 u_MvpMatrix = u_pMatrix * u_mvMatrix;
    gl_Position  = u_MvpMatrix * vec4(in_Position, 1.0);
    gl_PointSize = point_size;
    v_Color = vec4(in_Color, 1.0);
};
)";

const char* point_color_fs = R"(
in  vec4 v_Color;
out vec4 out_FragColor;

void main()
{
    vec2 pp = gl_PointCoord - vec2(0.5, 0.5);
    if (dot(pp, pp) > 0.25) discard;
    out_FragColor = v_Color;
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

uniform mat4 mProj;
uniform mat4 mView;
uniform float uVectorSize;

in VertexData
{
    vec3 position;
    vec3 direction;
} v_data[];

void main()
{
    vec3 p = v_data[0].position;
    vec3 n = p + uVectorSize * normalize(v_data[0].direction);

    gl_Position = mProj * mView * vec4(p, 1.0);
    EmitVertex();

    gl_Position = mProj * mView * vec4(n, 1.0);
    EmitVertex();

    EndPrimitive();
}
)";

const char* vector_fs = R"(
out vec4 f_color;
 
uniform vec3 u_color;

void main()
{
    f_color = vec4(u_color, 1.0);
}
)";


const char* cylinder_vs = R"(
layout (location = 0) in vec3 v_position;
layout (location = 1) in vec3 v_direction;
layout (location = 2) in float v_curvature_radius;
layout (location = 3) in float v_height;

out VertexData
{
    vec3 position;
    vec3 direction;
    float radius;
    float height;
};

void main()
{
    position = v_position;
    direction = v_direction;
    radius = v_curvature_radius;
    height = v_height;
}
)";

const char* cylinder_gs = R"(
#define CIRCLE_SUBDIVISIONS 20

layout (points) in;
layout (triangle_strip, max_vertices=40) out;

uniform mat4 mProj;
uniform mat4 mView;

in VertexData
{
    vec3 position;
    vec3 direction;
    float radius;
    float height;
} v_data[];

out vec3 v_normal;


vec3 random_perpendicular(vec3 v) {
    return v.z < v.x ? vec3(v.y,-v.x,0) : vec3(0,-v.z,v.y);
}

mat4 rotationMatrix(vec3 axis, float angle) {
    float s = sin(angle);
    float c = cos(angle);
    float oc = 1.0 - c;
    
    return mat4(oc * axis.x * axis.x + c,           oc * axis.x * axis.y - axis.z * s,  oc * axis.z * axis.x + axis.y * s,  0.0,
                oc * axis.x * axis.y + axis.z * s,  oc * axis.y * axis.y + c,           oc * axis.y * axis.z - axis.x * s,  0.0,
                oc * axis.z * axis.x - axis.y * s,  oc * axis.y * axis.z + axis.x * s,  oc * axis.z * axis.z + c,           0.0,
                0.0,                                0.0,                                0.0,                                1.0);
}

vec3 rotate(vec3 v, vec3 axis, float angle) {
    mat4 m = rotationMatrix(axis, angle);
    return (m * vec4(v, 1.0)).xyz;
}

void main()
{
    vec3 p = v_data[0].position;
    vec3 axis = normalize(v_data[0].direction);
    vec3 r = v_data[0].radius * normalize(random_perpendicular(axis));

    vec4 cap_offset = vec4(axis * v_data[0].height, 0.0);

    for (int i = 0; i <= CIRCLE_SUBDIVISIONS; i++) {
        float ang = 3.14159265 * 2.0 / (CIRCLE_SUBDIVISIONS-1) * i;
        vec4 rot = vec4(rotate(r, axis, ang) + p, 1.0);

        gl_Position = mProj * mView * (rot + cap_offset);
        v_normal = rot.xyz;
        EmitVertex();
        gl_Position = mProj * mView * (rot - cap_offset);
        v_normal = rot.xyz;
        EmitVertex();
    }

    EndPrimitive();
}
)";

const char* cylinder_fs = R"(
out vec4 f_color;

in vec3 v_normal;

uniform vec3 u_color;
uniform mat4 mView;

void main()
{
    f_color = vec4(max(dot(vec3(0.0, 0.0, 1.0), normalize(vec3(v_normal))), 0.2) * u_color, 1.0);
}
)";
 
void TextRenderDrawList::add_text(const glm::vec2 &position, const char *text, size_t size)
{
    positions.push_back(position);
    begin.push_back(text_buffer.size());
    text_buffer += std::string_view(text, size);
}

void TextRenderDrawList::dump_to_draw_list(ImDrawList* list, glm::vec2 offset, glm::vec2 max)
{
    //list->PushClipRect(offset, size);
    glm::vec2 size = max -offset;
    
    begin.push_back(text_buffer.size());
    for (size_t i = 0; i < begin.size()-1; i++) {
        glm::vec2 p = (glm::vec2(1, 1) + positions[i]) * size * 0.5f;
        p.y = size.y - p.y;
        list->AddText(p + offset, 0xFFFFFFFF, &text_buffer[begin[i]], &text_buffer[begin[i+1]]);
    }
    //list->PopClipRect();

    text_buffer.clear();
    positions.clear();
    begin.clear();
}

ShaderCollection::ShaderCollection()
    : shaders(
        { gfx::ShaderProgram::Builder("Points")
                .register_class<gfx::CameraLens>("u_pMatrix")
                .register_class<gfx::CameraRig>("u_mvMatrix")
                .register_uniform<Color>("u_color")
                .register_uniform<PointSize>("point_size")
                .with_vertex_shader(point_vs)
                .with_fragment_shader(point_fs)
                .build_shared(),
            gfx::ShaderProgram::Builder("PointsColor")
                .register_class<gfx::CameraLens>("u_pMatrix")
                .register_class<gfx::CameraRig>("u_mvMatrix")
                .register_uniform<PointSize>("point_size")
                .with_vertex_shader(point_color_vs)
                .with_fragment_shader(point_color_fs)
                .build_shared(),
            gfx::ShaderProgram::Builder("Vectors")
                .register_class<gfx::CameraLens>("mProj")
                .register_class<gfx::CameraRig>("mView")
                .register_uniform<VectorSize>("uVectorSize")
                .register_uniform<Color>("u_color")
                .with_vertex_shader(vector_vs)
                .with_geometry_shader(vector_gs)
                .with_fragment_shader(vector_fs)
                .build_shared(),
            gfx::ShaderProgram::Builder("Cylinders")
                .register_class<gfx::CameraLens>("mProj")
                .register_class<gfx::CameraRig>("mView")
                .register_uniform<Color>("u_color")
                .with_vertex_shader(cylinder_vs)
                .with_geometry_shader(cylinder_gs)
                .with_fragment_shader(cylinder_fs)
                .build_shared()})
{
}

void ShaderCollection::init(entt::registry& reg)
{
    reg.set<ShaderCollection>();
}

void ShaderCollection::deinit(entt::registry &reg)
{
    reg.unset<ShaderCollection>();
}