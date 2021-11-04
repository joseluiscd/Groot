#pragma once
#include "command_gui.hpp"


class GraphVolume : public EntityCommand {
    
};


template <>
struct DefineGui<GraphVolume> {
    static constexpr std::string_view name = "Graph Volume";

    static void draw_gui(Command& c);
};
