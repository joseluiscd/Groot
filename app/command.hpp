#pragma once

#include <string>

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

    std::string error_string = "";
};
