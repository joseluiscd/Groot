#include "gfx/imgui/imgui_internal.h"
#include <gfx/imgui/gfx.hpp>
#include <gfx/imgui/imgui.h>
#include <groot_app/components.hpp>
#include <groot_app/entity_editor.hpp>
#include <groot_app/render.hpp>
#include <groot_app/resources.hpp>
#include <groot_app/viewer_system.hpp>
#include <spdlog/spdlog.h>

namespace viewer_system {

void run(entt::registry& registry)
{
    ImGui::SetNextWindowSize(ImVec2(200, 200), ImGuiCond_FirstUseEver);

    RenderData& data = registry.ctx<RenderData>();
    TextRenderDrawList& text = registry.ctx<TextRenderDrawList>();

    if (ImGui::BeginFramebuffer("3D Viewer", data.framebuffer)) {
        glm::ivec2 current_size = ImGui::GetWindowContentSize();
        if (current_size != data.size) {
            data.size = current_size;
            data.camera->lens().resize_event(current_size);
        }

        if (ImGui::IsItemActive()) {
            glm::vec2 drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left, 0.0);
            data.camera->orbit(drag.x * 0.01);
            data.camera->orbit_vertical(drag.y * 0.01);
            ImGui::ResetMouseDragDelta(ImGuiMouseButton_Left);

            drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right, 0.0);
            data.camera->truck(-drag.x * 0.01);
            data.camera->vertical(drag.y * 0.01);
            ImGui::ResetMouseDragDelta(ImGuiMouseButton_Right);
        }

        if (ImGui::IsItemHovered()) {
            data.camera->advance(ImGui::GetIO().MouseWheel * 0.2);
            // data.lens->zoom(1.0 + ImGui::GetIO().MouseWheel * 0.001);
        }

        text.dump_to_draw_list(ImGui::GetWindowDrawList(), ImGui::GetItemRectMin(), ImGui::GetItemRectMax());
        ImGui::EndFramebuffer();
    }
}

void init(entt::registry& registry)
{
    auto lens = std::make_unique<gfx::PerspectiveCameraLens>(70.0, 1.0, 0.2, 100.2);
    auto camera = std::make_unique<gfx::CameraRig>(*lens);

    RenderData data {
        gfx::Framebuffer(),
        std::move(camera),
        std::move(lens),
        glm::ivec2(0, 0)
    };

    data.framebuffer
        .add_color_buffer(glm::ivec2(2048, 2048), gfx::TextureType::Rgba, 4)
        .set_depth_buffer(glm::ivec2(2048, 2048), 4);

    data.camera
        ->with_position({ 0.0, 0.0, 20.0 })
        .look_at({ 0.0, 0.0, 0.0 })
        .with_up_vector({ 0.0, 1.0, 0.0 });

    registry.set<RenderData>(std::move(data));
    registry.set<TextRenderDrawList>();
}

void deinit(entt::registry& registry)
{
    registry.unset<RenderData>();
}

#define SNAKE_MAP_SIZE 32

#include <GLFW/glfw3.h>
void python()
{
    static bool snake_mode = true;
    static bool pre_snake_mode = false;
    static bool a = rand() % 2;
    static bool b = rand() % 2;
    static bool c = rand() % 2;
    static bool d = rand() % 2;
    static bool e = rand() % 2;
    static bool f = rand() % 2;
    static bool g = rand() % 2;
    static bool h = rand() % 2;
    static bool i = rand() % 2;
    static bool j = rand() % 2;

    ImGuiIO& io = ImGui::GetIO();

    if (io.KeyCtrl && io.KeyAlt && io.KeyShift && ImGui::IsKeyPressed(GLFW_KEY_F2)) {
        pre_snake_mode = true;
    }

    if (pre_snake_mode) {
        ImGui::Begin("Activate python mode", &pre_snake_mode);
        ImGui::Checkbox("Enable Checkbox automatic operations", &d);
        ImGui::Checkbox("Enable Semantic segmentation", &e);
        ImGui::Checkbox("Enable Tree clustering", &f);
        ImGui::Checkbox("Enable Ground detection", &b);
        ImGui::Checkbox("Enable Python bindings", &a);
        ImGui::Checkbox("Enable GEU interoperability", &g);
        ImGui::Checkbox("Enable Olive tree special mode", &h);
        ImGui::Checkbox("Enable Point cloud normalization", &i);
        ImGui::Checkbox("Enable Fruit detection", &c);
        ImGui::Checkbox("Enable Interface for experimental applications", &j);
        ImGui::End();

        if (a && b && c && d && !e && !f && !g && !h && !i && !j) {
            snake_mode = true;
            pre_snake_mode = false;
            a = rand() % 2;
            b = rand() % 2;
            c = rand() % 2;
            d = rand() % 2;
            e = rand() % 2;
            f = rand() % 2;
            g = rand() % 2;
            h = rand() % 2;
            i = rand() % 2;
            j = rand() % 2;
        }
    }
    if (!snake_mode) {
        return;
    }

    static glm::ivec2 food_position { rand() % SNAKE_MAP_SIZE, rand() % SNAKE_MAP_SIZE };
    static std::list<glm::ivec2> snake { { 5, 2 }, { 4, 2 }, { 3, 2 }, { 2, 2 } };
    static size_t framerate = 6;
    static glm::ivec2 direction = { 1, 0 };
    static const int tile_size { 16 };
    static bool lost = false;

    int lr = (direction.x == 0) * (int(ImGui::IsKeyPressed(GLFW_KEY_RIGHT, false)) - int(ImGui::IsKeyPressed(GLFW_KEY_LEFT, false)));
    int ud = (direction.y == 0) * (int(ImGui::IsKeyPressed(GLFW_KEY_DOWN, false)) - int(ImGui::IsKeyPressed(GLFW_KEY_UP, false)));

    direction.x = direction.x * (ud == 0) + lr;
    direction.y = direction.y * (lr == 0) + ud;

    direction.x = int(0 < direction.x) - int(direction.x < 0);
    direction.y = int(0 < direction.y) - int(direction.y < 0);

    if (ImGui::GetFrameCount() % framerate == 0 && !lost) {
        glm::ivec2 head = snake.front();

        glm::ivec2 new_head = head + direction;

        new_head.x = (new_head.x + SNAKE_MAP_SIZE) % SNAKE_MAP_SIZE;
        new_head.y = (new_head.y + SNAKE_MAP_SIZE) % SNAKE_MAP_SIZE;

        if (new_head == food_position) {
            food_position = { rand() % SNAKE_MAP_SIZE, rand() % SNAKE_MAP_SIZE };
        } else if (std::find(snake.begin(), snake.end(), new_head) != snake.end()) {
            lost = true;
        } else {
            snake.pop_back();
        }
        snake.push_front(new_head);
    }

    if(ImGui::Begin("Python Snake", &snake_mode, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoDocking)) {
    ImGuiWindow* window = ImGui::GetCurrentWindow();
        ImVec2 cursor = window->DC.CursorPos;
        const ImRect rect { { cursor.x, cursor.y }, { cursor.x + tile_size * SNAKE_MAP_SIZE, cursor.y + tile_size * SNAKE_MAP_SIZE } };
        ImGui::ItemSize(rect);
        ImGui::ItemAdd(rect, window->GetID("snake_map"));

        for (auto it = snake.begin(); it != snake.end(); ++it) {
            int x = it->x;
            int y = it->y;

            ImVec2 pos_min { cursor.x + x * tile_size, cursor.y + y * tile_size };
            ImVec2 pos_max { cursor.x + (x + 1) * tile_size, cursor.y + (y + 1) * tile_size };

            window->DrawList->AddRectFilled(pos_min, pos_max, ImGui::GetColorU32(ImGuiCol_Button), 4.0, 0);
        }

        ImVec2 food_center { cursor.x + (0.5f + food_position.x) * tile_size, cursor.y + (0.5f + food_position.y) * tile_size };
        window->DrawList->AddCircleFilled(food_center, tile_size / 2.0f, ImGui::GetColorU32(ImGuiCol_ButtonActive));
    }
    ImGui::End();
}

}