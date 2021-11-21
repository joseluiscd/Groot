#pragma once

#include "command.hpp"
#include <memory>
#include <spdlog/spdlog.h>
#include "task.hpp"

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
    virtual void schedule_commands(entt::registry& reg) = 0;
    virtual GuiResult draw_gui() = 0;
    virtual ~Gui() { }

    template <typename Cmd, typename Iterator, typename Function>
    static std::vector<Command*> command_to_entities(entt::registry& reg, Iterator begin, Iterator end, Function&& f);
};

class DialogGui : public Gui {
public:
    virtual ~DialogGui() { }
    virtual GuiResult draw_gui() override;
    virtual void draw_dialog() = 0;
    virtual std::string_view name() const = 0;
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

    void schedule_commands(entt::registry& reg) override
    {
        std::string name(gui->name());

        auto&& task = async::spawn(async_scheduler(), [gui = std::exchange(gui, nullptr)]() {
            std::unique_ptr<CommandGui> cmd(gui);
            if (cmd->execute() == CommandState::Error) {
                throw std::runtime_error(cmd->error_string);
            };
            return cmd;
        }).then(sync_scheduler(), [&reg](std::unique_ptr<CommandGui>&& cmd){
            cmd->on_finish(reg);
        });

        reg.ctx<TaskBroker>().push_task(name, std::move(task));
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

template <typename Cmd, typename Iterator, typename Function>
std::vector<Command*> Gui::command_to_entities(entt::registry& reg, Iterator begin, Iterator end, Function&& f)
{
    static_assert(std::is_base_of_v<Command, Cmd>, "Cmd must be base of Command");
    static_assert(std::is_same_v<typename std::iterator_traits<Iterator>::value_type, entt::entity>,
        "`Iterator end` must be an entity iterator");
    static_assert(std::is_same_v<std::invoke_result_t<decltype(f), entt::handle>, Cmd*>, 
        "`Function&& f` must have function signature `Cmd* f(entt::handle h)`");
    
    std::vector<Command*> result;
    std::transform(begin, end, std::back_inserter(result), [f = std::move(f), &reg](entt::entity entity){
        return (Command*) f(entt::handle(reg, entity));
    });

    return result;
}