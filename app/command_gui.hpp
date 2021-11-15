#pragma once

#include "command.hpp"

enum class GuiState {
    Close = 0,
    Editing,
    RunAsync,
    RunSync,
    RunAsyncUpdate
};

class CommandGui : public Command {
public:
    virtual GuiState draw_gui() = 0;
};