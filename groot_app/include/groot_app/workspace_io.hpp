#pragma once

#include <filesystem>
#include <groot_app/command_gui.hpp>
#include <groot_app/entt.hpp>

GROOT_APP_API async::task<void> open_workspace_command(entt::registry& reg, const std::string& filename);
GROOT_APP_API async::task<void> save_workspace_command(const entt::registry& reg, const std::string& filename);

class GROOT_APP_LOCAL OpenWorkspace : public FileDialogGui {
public:
    OpenWorkspace();

    void schedule_commands(entt::registry& reg, const std::string& filename) override;
};

class GROOT_APP_LOCAL SaveWorkspace : public FileDialogGui {
public:
    SaveWorkspace();

    void schedule_commands(entt::registry& reg, const std::string& filename) override;
};
