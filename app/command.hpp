#pragma once

#include <string>
#include <stdexcept>

enum class CommandState : bool {
    Ok = false,
    Error = true,
};

/// Abstract command
class Command {
public:
    virtual ~Command() { }
    virtual CommandState execute() = 0;
    virtual void on_finish() {}

    CommandState run() {
        if (this->execute() == CommandState::Ok) {
            this->on_finish();
            return CommandState::Ok;
        }
        return CommandState::Error;
    }

    virtual const std::string_view name() {
        return "Generic command name";
    }

    std::string error_string = "";
};

inline void throw_on_error(CommandState state, const std::string& error)
{
    if (state != CommandState::Ok)
        throw std::runtime_error(error);
}