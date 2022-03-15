#pragma once

#include <gfx/imgui/imfilebrowser.h>
#include <groot_app/groot_app.hpp>
#include <groot_app/task.hpp>
#include <memory>
#include <spdlog/spdlog.h>

enum class GuiResult {
    KeepOpen = 0,
    Close,
    RunAndClose,
    RunAndKeepOpen,
};

/**
 * Interface that manages the Gui Window.
 */
class GROOT_APP_LOCAL Gui {
public:
    virtual void schedule_commands(entt::registry& reg) = 0;
    virtual GuiResult draw_gui() = 0;
    virtual ~Gui() { }
};

/**
 * Simple Dialog Interface. Manages the Gui.
 * Override the `draw_dialog` method.
 */
class GROOT_APP_API DialogGui : public Gui {
public:
    virtual ~DialogGui() { }
    virtual GuiResult draw_gui() override;

    /// Draw the dialog contents. No need to create windows, just widgets.
    virtual void draw_dialog() = 0;
    virtual std::string_view name() const = 0;
};

enum class FileDialogType {
    Open,
    Save
};

/**
 * Simple File Dialog Interface. Manages the Gui.
 * Override the `draw_dialog` method.
 */
class GROOT_APP_API FileDialogGui : public Gui {
public:
    FileDialogGui(
        FileDialogType type,
        const std::string& name,
        const std::vector<std::string>& typeFilters);

    virtual ~FileDialogGui() { }

    virtual GuiResult draw_gui() override;
    void schedule_commands(entt::registry& reg) override
    {
        schedule_commands(reg, selected_file.string());
    }

    virtual void schedule_commands(entt::registry& reg, const std::string& filename) = 0;

private:
    ImGui::FileBrowser file_dialog;
    std::filesystem::path selected_file;
};
