#pragma once

#include "command.hpp"
#include <memory>
#include <spdlog/spdlog.h>

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

class CommandGui : public Command {
public:
    virtual GuiState draw_gui() = 0;
};

class Gui {
public:
    virtual std::vector<Command*> get_commands() = 0;
    virtual GuiResult draw_gui() = 0;
    virtual ~Gui() {}
};

class GuiAdapter : public Gui {
public:
    template <typename CmdGui>
    GuiAdapter(CmdGui* _gui)
        : gui(_gui)
    {
        static_assert(std::is_base_of_v<CommandGui, CmdGui>, "Template parameter must be a legacy `CommandGui`");
        spdlog::warn("This command should use the new interface.");
    }

    std::vector<Command*> get_commands() override
    {
        return { (Command*)gui };
    }

    GuiResult draw_gui() override
    {
        switch (gui->draw_gui()) {
        case GuiState::Close:
            return GuiResult::Close;
        case GuiState::Editing:
            return GuiResult::KeepOpen;
        case GuiState::RunAsync:
            return GuiResult::RunAndClose;
        //case GuiState::RunSync:
        //    return GuiResult::RunAndClose;
        case GuiState::RunAsyncUpdate:
            return GuiResult::RunAndKeepOpen;
        default:
            return GuiResult::KeepOpen;
        }
    }

private:
    CommandGui* gui;
};

template <typename CmdGui, typename... Args>
GuiAdapter* make_gui_adapter(Args&&... args)
{
    return new GuiAdapter(new CmdGui(std::forward<Args...>(args)...));
}
