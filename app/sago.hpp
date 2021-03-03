#pragma once
#include "command_gui.hpp"
#include "data_source.hpp"
#include "data_output.hpp"
#include <groot/plant_graph.hpp>

class SagoPreprocess : public CommandGui {
public:
    SagoPreprocess(IDataSource<groot::PlantGraph>);

    GuiState draw_gui() override;
    CommandState execute() override;
};