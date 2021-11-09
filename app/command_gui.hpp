#pragma once

#include "command.hpp"
#include "components.hpp"
#include <async++.h>
#include <entt/entt.hpp>
#include <future>
#include <gfx/imgui/imgui.h>
#include <type_traits>

enum class GuiState {
    Close = 0,
    Editing,
    RunAsync,
    RunSync,
    RunAsyncUpdate
};

/// An optional error
using Result = std::optional<std::string>;

class CommandGui : public Command {
public:
    virtual GuiState draw_gui() = 0;
};

class GenericCommand {
public:
    GenericCommand();

    virtual Result async_do(const entt::registry& reg) = 0;
    virtual void finish(entt::registry& reg) = 0;

    template <typename OutStream>
    void serialize(OutStream& o);

    /// Run the task in the current thread synchronously
    Result operator()(entt::registry& reg)
    {
        auto result = this->async_do(reg);
        if (!result) {
            this->finish(reg);
        }
        return result;
    }

    template <typename SchedulerAsync, typename SchedulerSync>
    async::task<Result> run_async(entt::registry& reg, SchedulerAsync& async_s, SchedulerSync& sync_s)
    {
        const entt::registry& const_reg = reg;
        return async::spawn(async_s, [this, &const_reg]() {
            return this->async_do(const_reg);
        }).then(sync_s, [this, &reg](Result&& res) {
            if (!res) {
                this->finish(reg);
            }
            return res;
        });
    }
};

class EntityCommand {
public:
    EntityCommand();

    virtual Result async_do(const entt::handle& h) = 0;
    virtual void finish(entt::handle& h) = 0;

    Result operator()(entt::handle& h)
    {
        auto result = this->async_do(h);
        if (!result) {
            this->finish(h);
        }
        return result;
    }

    template <typename SchedulerAsync, typename SchedulerSync>
    async::task<Result> run_async(entt::handle& h, SchedulerAsync& async_s, SchedulerSync& sync_s)
    {
        const entt::handle& const_handle = h;
        return async::spawn(async_s, [this, &const_handle]() {
            return this->async_do(const_handle);
        }).then(sync_s, [this, &h](Result&& res) {
            if (!res) {
                this->finish(h);
            }
            return res;
        });
    }
};

template <typename T>
class OldCommand : public GenericCommand {
public:
    static_assert(std::is_convertible_v<T, Command>, "Template argument must be a Command");

    OldCommand(T&& _t)
        : t(_t)
    {
    }

    template <typename... Args>
    OldCommand(Args&&... args)
        : t(std::forward<Args>(args)...)
    {
    }

    Result async_do(const entt::registry& reg) override
    {
        if (t.execute() == CommandState::Ok) {
            return {};
        } else {
            return t.error_string;
        }
    }
    void finish(entt::registry& reg) override
    {
        t.on_finish();
    }

private:
    T t;
};

template <typename Command>
struct Gui {
    static GuiState run_gui(Command& c);
};

struct AbstractGui {
    virtual GuiState run_gui() = 0;
};

template <typename Cmd>
struct GuiRunner : public AbstractGui {
    GuiRunner(Cmd&& _c)
        : c(_c)
    {
    }

    template <typename... Args>
    GuiRunner(Args&&... args)
        : c(std::forward(args)...)
    {
    }

    GuiState run_gui() override
    {
        return Gui<Cmd>::run_gui(c);
    }

    Cmd c;
};

template <typename Command>
struct DefineGui {
    static constexpr std::string_view name = "";
    static void draw_gui(Command& c);
};

template <typename Command>
struct HasGuiTrait {
    template <typename U>
    static auto test(U*) -> std::integral_constant<bool, sizeof(DefineGui<U>) == sizeof(DefineGui<U>)>;
    static auto test(...) -> std::false_type;

    using type = decltype(test((Command*)0));
    using value = typename type::value;
};

template <typename Cmd>
class CommandGuiLegacyWrapper : public CommandGui {
    static_assert(std::is_convertible_v<Cmd, GenericCommand>, "Cmd must be a Generic Command");
    static_assert(std::is_default_constructible_v<Cmd>, "Cmd must be default constructible");

public:
    CommandGuiLegacyWrapper(entt::handle&& _handle)
        : command()
        , handle(_handle)
    {
    }

    CommandGuiLegacyWrapper(entt::registry& _reg)
        : CommandGuiLegacyWrapper(entt::handle {
            _reg,
            _reg.ctx<SelectedEntity>().selected })
    {
    }

    virtual GuiState draw_gui()
    {
        return Gui<Cmd>::run_gui(command);
    }
    virtual CommandState execute()
    {
        return command.stage1(*handle.registry());
    };

    virtual void on_finish()
    {
        command.stage2(*handle.registry());
    }

private:
    Cmd command;
    entt::handle handle;
};

template <typename Command>
GuiState Gui<Command>::run_gui(Command& c)
{
    if constexpr (!HasGuiTrait<Command>::value) {
        return GuiState::RunAsync;
    }

    bool show = true;
    ImGui::OpenPopup(DefineGui<Command>::name.data());
    if (ImGui::BeginPopupModal(DefineGui<Command>::name.data(), &show, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
        DefineGui<Command>::draw_gui(c);

        ImGui::Separator();

        if (ImGui::Button("Run")) {
            //All configured, run the command
            ImGui::EndPopup();
            return GuiState::RunAsync;
        }

        ImGui::EndPopup();
    }

    return show ? GuiState::Editing : GuiState::Close;
}