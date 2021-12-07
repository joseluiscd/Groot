#pragma once

#include <groot/groot.hpp>
#include <groot_app/entt.hpp>
#include <stdexcept>
#include <string>

enum class CommandState : bool {
    Ok = false,
    Error = true,
};

/// Abstract command
class GROOT_LOCAL Command {
public:
    virtual ~Command() { }
    virtual CommandState execute() = 0;
    virtual void on_finish(entt::registry& reg) { }

    CommandState run(entt::registry& reg)
    {
        if (this->execute() == CommandState::Ok) {
            this->on_finish(reg);
            return CommandState::Ok;
        }
        return CommandState::Error;
    }

    virtual const std::string_view name() const
    {
        return "Generic command name";
    }

    std::string error_string = "";
};

inline GROOT_LOCAL void throw_on_error(CommandState state, const std::string& error)
{
    if (state != CommandState::Ok)
        throw std::runtime_error(error);
}
