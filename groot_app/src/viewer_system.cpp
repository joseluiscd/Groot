#include <gfx/imgui/gfx.hpp>
#include <gfx/imgui/imgui.h>
#include <groot_app/components.hpp>
#include <groot_app/entity_editor.hpp>
#include <groot_app/render.hpp>
#include <groot_app/resources.hpp>
#include <groot_app/viewer_system.hpp>

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
#define SNAKE_FRAMERATE 

void snake()
{
    ImGuiIO& io = ImGui::GetIO();

    static bool enabled = true;

    static glm::ivec2 food_position { rand() % SNAKE_MAP_SIZE, rand() % SNAKE_MAP_SIZE };
    static std::list<glm::ivec2> snake { {5, 2}, {4, 2}, {3, 2}, {2, 2}};
    static size_t framerate = 6;
    static glm::ivec2 direction = {1, 0};
    static int snake_table[SNAKE_MAP_SIZE * SNAKE_MAP_SIZE];

    static const int key_a = ImGui::GetKeyIndex(ImGuiKey_LeftArrow);
    static const int key_s = ImGui::GetKeyIndex(ImGuiKey_DownArrow);
    static const int key_d = ImGui::GetKeyIndex(ImGuiKey_RightArrow);
    static const int key_w = ImGui::GetKeyIndex(ImGuiKey_UpArrow);

    ImGui::Begin("Snake");
    if (ImGui::GetFrameCount() % framerate == 0) {
        glm::ivec2 head = snake.front();

        glm::ivec2 new_head = head + direction;

        if (new_head.x >= SNAKE_MAP_SIZE) {
            new_head.x = 0;
        }

        if (new_head.x < 0) {
            new_head.x = SNAKE_MAP_SIZE - 1;
        }

        if (new_head.y >= SNAKE_MAP_SIZE) {
            new_head.y = 0;
        }

        if (new_head.y < 0) {
            new_head.y = 0;
        }

        snake.push_front(new_head);

        for (size_t i = 0; i < SNAKE_MAP_SIZE; ++i) {
            for (size_t j = 0; j < SNAKE_MAP_SIZE; ++j) {
                snake_table[i * SNAKE_MAP_SIZE + j] = 0;
            }
        }

        for (auto it = snake.begin(); it != snake.end(); ++it) {
            snake_table[it->x * SNAKE_MAP_SIZE + it->y] = 1; 
        }
        snake_table[food_position.x * SNAKE_MAP_SIZE + food_position.y] = -1;

        if (new_head == food_position) {
            bool is_on_snake = false;
            do {
                food_position = { rand() % SNAKE_MAP_SIZE, rand() % SNAKE_MAP_SIZE};
                is_on_snake = false;
                for (auto it = snake.begin(); it != snake.end(); ++it) {
                    if (*it == food_position) {
                        is_on_snake = true;
                    }
                }
            } while (is_on_snake);
        } else {
            snake.pop_back();
        }

        if (direction.y == 0) {
            if (ImGui::IsKeyPressed(key_a)) {
                direction = {-1, 0};
            }
            if (ImGui::IsKeyPressed(key_d)) {
                direction = {1, 0};
            }
        }

        if (direction.x == 0) {
            if (ImGui::IsKeyPressed(key_s)) {
                direction = {0, -1};
            }
            if (ImGui::IsKeyPressed(key_w)) {
                direction = {0, 1};
            }
        }
    }

    for (size_t i = 0; i < SNAKE_MAP_SIZE; ++i) {
        for (size_t j = 0; j < SNAKE_MAP_SIZE; ++j) {
            int v = snake_table[i * SNAKE_MAP_SIZE + j];
            ImGui::PushID(i * SNAKE_MAP_SIZE + j);
            bool activate = false;
            if (v == 0) {
                ImGui::BeginDisabled();
                ImGui::Checkbox("", &activate);
                ImGui::EndDisabled();
            } else if (v > 0) {
                ImGui::Checkbox("", &activate);
            } else if (v < 0) {
                activate = true;
                ImGui::Checkbox("", &activate);
            }
            ImGui::SameLine();
            ImGui::PopID();
        }
        ImGui::NewLine();
    }
    ImGui::End();
}

}