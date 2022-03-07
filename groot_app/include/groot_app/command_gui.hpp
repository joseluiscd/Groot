#pragma once

#include <groot_app/groot_app.hpp>
#include <groot_app/command.hpp>
#include <memory>
#include <spdlog/spdlog.h>
#include <groot_app/task.hpp>

/// Legacy
enum class GuiState {
    Close = 0,
    Editing,
    RunAsync,
    RunAsyncUpdate
};

/// New
enum class GuiResult {
    KeepOpen = 0,
    Close,
    RunAndClose,
    RunAndKeepOpen,
};

class GROOT_APP_LOCAL CommandGui : public Command {
public:
    virtual GuiState draw_gui() = 0;
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

class GROOT_APP_API GuiAdapter : public Gui {
public:
    template <typename CmdGui>
    GuiAdapter(CmdGui* _gui)
        : gui(_gui)
    {
        static_assert(std::is_base_of_v<CommandGui, CmdGui>, "Template parameter must be a legacy `CommandGui`");
        spdlog::warn("This command should use the new interface.");
    }

    void schedule_commands(entt::registry& reg) override;
    GuiResult draw_gui() override;

private:
    CommandGui* gui;
};

template <typename CmdGui, typename... Args>
GuiAdapter* make_gui_adapter(Args&&... args)
{
    return new GuiAdapter(new CmdGui(std::forward<Args...>(args)...));
}