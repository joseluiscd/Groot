#include "editor.hpp"
#include <gfx/font_awesome.hpp>
#include <spdlog/spdlog.h>
#include <fmt/format.h>

Editor::Editor(lua_State* _L)
    : editor(std::make_unique<Zep::ZepEditor_ImGui>("", Zep::NVec2f(1.0)))
    , L(_L)
{
    editor->SetGlobalMode(Zep::ZepMode_Vim::StaticName());
    auto& display = (Zep::ZepDisplay_ImGui&)editor->GetDisplay();
    display.SetFont(Zep::ZepTextType::Text, std::make_shared<Zep::ZepFont_ImGui>(display, ImGui::GetIO().Fonts->Fonts[1], int(13)));
    display.SetFont(Zep::ZepTextType::UI, std::make_shared<Zep::ZepFont_ImGui>(display, ImGui::GetIO().Fonts->Fonts[1], int(13)));
}

void Editor::draw_menu_bar()
{
    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem(ICON_FA_FOLDER_OPEN "\tOpen...")) {
            }
            if (ImGui::MenuItem(ICON_FA_SAVE "\tSave")) {
            }
            if (ImGui::MenuItem(ICON_FA_SAVE "\tSave as...")) {
            }
            ImGui::EndMenu();
        }

        if (ImGui::MenuItem(ICON_FA_PLAY "\tRun")) {
            auto& buffer = editor->GetEditor().GetActiveTabWindow()->GetActiveWindow()->GetBuffer().GetWorkingBuffer();
            std::string data(buffer.begin(), buffer.end());
            const char* miau = data.data();
            int return_val = luaL_dostring(L, data.data());
            if (return_val != 0) {
                spdlog::error("Lua error: {}", lua_tostring(L, -1));
            }
        }

        ImGui::EndMenuBar();
    }
}

bool Editor::render()
{
    bool open = true;
    ImGui::SetNextWindowSizeConstraints(ImVec2(200, 200), ImVec2(2048, 2048));
    std::string title = fmt::format("Editor##{}", (size_t) this);
    ImGui::Begin(title.c_str(), &open, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_MenuBar);
    draw_menu_bar();

    auto min = ImGui::GetCursorScreenPos();
    auto max = ImGui::GetContentRegionAvail();
    max.x = std::max(1.0f, max.x);
    max.y = std::max(1.0f, max.y);

    max.x = min.x + max.x;
    max.y = min.y + max.y;
    editor->SetDisplayRegion(Zep::NVec2f(min.x, min.y), Zep::NVec2f(max.x, max.y));

    editor->Display();
    editor->HandleInput();

    ImGui::End();
    return open;
}