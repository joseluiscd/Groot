#include <groot_app/screenshot.hpp>
#include <gfx/framebuffer.hpp>
#include <lodepng/lodepng.hpp>
#include <groot_app/render.hpp>
#include <glm/glm.hpp>
#include <groot_app/task.hpp>
#include <algorithm>
#include <utility>

void take_screenshot(entt::registry& reg, const std::string& filename)
{
    RenderData& render_data = reg.ctx<RenderData>();
    TaskBroker& task_broker = reg.ctx<TaskBroker>();

    gfx::Framebuffer screenshot;
    gfx::TextureHandle colors;
    colors.reserve_size(render_data.size, 1, gfx::TextureFormat::Rgba);

    screenshot.add_color_buffer(colors);
    
    render_data.framebuffer.blit_colors(screenshot, { 0, 0 }, render_data.size, { 0, 0 }, render_data.size);

    std::vector<uint8_t> data = colors.read_colors();

    task_broker.push_task(
        "Saving screenshot",
        async::spawn(async_scheduler(), [filename, data = colors.read_colors(), size = render_data.size]() {
            /* for (size_t j = 0; j < size.y / 2; j++) {
                for (size_t i = 0; i < size.x; i++) {
                    for (size_t k = 0; k < 4; k++) {
						std::swap<uint8_t>(data[k + 4 * ( i + size.x * j )], data[k + 4 * ( i + size.x * (size.y - j) )]);
					}
                }
            }*/
            lodepng::encode(filename, data, size.x, size.y, LCT_RGBA, 8U);
        })
    );

}
