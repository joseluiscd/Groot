#pragma once

#include <gfx/imgui/imgui.h>
#include <groot/cgal.hpp>
#include <groot_app/command_gui.hpp>
#include <groot_app/components.hpp>
#include <groot_app/entt.hpp>
#include <optional>

GROOT_APP_API async::task<entt::entity> import_graph_command(entt::registry& reg, const std::string_view& file);
GROOT_APP_API async::task<void> export_graph_command(entt::handle e, const std::string_view& file);

class GROOT_APP_LOCAL ImportGraphGui : public FileDialogGui {
public:
    ImportGraphGui();
    void schedule_commands(entt::registry& reg, const std::string& filename) override;
};

class GROOT_APP_LOCAL ExportGraphGui : public FileDialogGui {
public:
    ExportGraphGui(entt::handle h);
    void schedule_commands(entt::registry& reg, const std::string& filename) override;

private:
    entt::handle target;
};