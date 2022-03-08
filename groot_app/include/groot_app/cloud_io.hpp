#pragma once

#include <groot/groot.hpp>
#include <groot_app/command_gui.hpp>
#include <groot_app/components.hpp>
#include <groot_app/entt.hpp>
#include <gfx/imgui/imgui.h>
#include <groot/cgal.hpp>
#include <optional>

GROOT_APP_API async::task<entt::entity> import_ply_command(entt::registry& reg, const std::string_view& file);
GROOT_APP_API async::task<void> export_ply_command(entt::handle e, const std::string_view& file);

class GROOT_APP_LOCAL ImportPLYGui : public FileDialogGui {
public:
    ImportPLYGui();
    void schedule_commands(entt::registry& reg, const std::string& filename) override;
};

class GROOT_APP_LOCAL ExportPLYGui : public FileDialogGui {
public:
    ExportPLYGui(entt::handle h);

    void schedule_commands(entt::registry& reg, const std::string& filename) override;

private:
    entt::handle target;
};