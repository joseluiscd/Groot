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

    std::string error_string = "";
};
