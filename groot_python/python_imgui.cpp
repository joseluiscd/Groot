#include "python_imgui.hpp"
#include <gfx/imgui/imgui.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

struct ImGuiModule {
};

void create_table_functions(py::module_& m)
{
    m.def("BeginTable", &ImGui::BeginTable,
        "name"_a,
        "columns"_a,
        "flags"_a = 0,
        "size"_a = ImVec2(0.0, 0.0),
        "inner_width"_a = 0.0,
        "Begin a table");

    m.def("EndTable", &ImGui::EndTable);
    m.def("TableNextColumn", &ImGui::TableNextColumn);
    m.def("TableNextRow", &ImGui::TableNextRow, "row_flags"_a = 0, "min_height"_a = 0.0f);
    m.def("TableSetupColumn", &ImGui::TableSetupColumn, "label"_a, "flags"_a = 0, "init_width_or_weight"_a = 0.0f, "user_id"_a = 0);
    m.def("TableHeadersRow", &ImGui::TableHeadersRow);

    py::enum_<ImGuiTableFlags_>(m, "TableFlags")
        .value("None", ImGuiTableFlags_None)
        .value("Resizable", ImGuiTableFlags_Resizable)
        .value("Reorderable", ImGuiTableFlags_Reorderable)
        .value("Hideable", ImGuiTableFlags_Hideable)
        .value("Sortable", ImGuiTableFlags_Sortable)
        .value("NoSavedSettings", ImGuiTableFlags_NoSavedSettings)
        .value("ContextMenuInBody", ImGuiTableFlags_ContextMenuInBody)
        .value("RowBg", ImGuiTableFlags_RowBg)
        .value("BordersInnerH", ImGuiTableFlags_BordersInnerH)
        .value("BordersOuterH", ImGuiTableFlags_BordersOuterH)
        .value("BordersInnerV", ImGuiTableFlags_BordersInnerV)
        .value("BordersOuterV", ImGuiTableFlags_BordersOuterV)
        .value("BordersH", ImGuiTableFlags_BordersH)
        .value("BordersV", ImGuiTableFlags_BordersV)
        .value("BordersInner", ImGuiTableFlags_BordersInner)
        .value("BordersOuter", ImGuiTableFlags_BordersOuter)
        .value("Borders", ImGuiTableFlags_Borders)
        .value("NoBordersInBody", ImGuiTableFlags_NoBordersInBody)
        .value("NoBordersInBodyUntilResize", ImGuiTableFlags_NoBordersInBodyUntilResize)
        .value("SizingFixedFit", ImGuiTableFlags_SizingFixedFit)
        .value("SizingFixedSame", ImGuiTableFlags_SizingFixedSame)
        .value("SizingStretchProp", ImGuiTableFlags_SizingStretchProp)
        .value("SizingStretchSame", ImGuiTableFlags_SizingStretchSame)
        .value("NoHostExtendX", ImGuiTableFlags_NoHostExtendX)
        .value("NoHostExtendY", ImGuiTableFlags_NoHostExtendY)
        .value("NoKeepColumnsVisible", ImGuiTableFlags_NoKeepColumnsVisible)
        .value("PreciseWidths", ImGuiTableFlags_PreciseWidths)
        .value("NoClip", ImGuiTableFlags_NoClip)
        .value("PadOuterX", ImGuiTableFlags_PadOuterX)
        .value("NoPadOuterX", ImGuiTableFlags_NoPadOuterX)
        .value("NoPadInnerX", ImGuiTableFlags_NoPadInnerX)
        .value("ScrollX", ImGuiTableFlags_ScrollX)
        .value("ScrollY", ImGuiTableFlags_ScrollY)
        .value("SortMulti", ImGuiTableFlags_SortMulti)
        .value("SortTristate", ImGuiTableFlags_SortTristate)
        ;
}

void create_imgui_module(py::module_& m)
{
    py::class_<ImVec2>(m, "Vec2")
        .def(py::init<>())
        .def(py::init<float, float>())
        .def_readwrite("x", &ImVec2::x)
        .def_readwrite("y", &ImVec2::y);

    py::class_<ImVec4>(m, "Vec4")
        .def(py::init<>())
        .def(py::init<float, float, float, float>())
        .def_readwrite("x", &ImVec4::x)
        .def_readwrite("y", &ImVec4::y)
        .def_readwrite("z", &ImVec4::z)
        .def_readwrite("w", &ImVec4::w);

    m.def(
        "Begin", [](const char* name, bool closable, int flags) {
            bool open = true;

            return py::make_tuple(ImGui::Begin(name, closable ? &open : nullptr, flags), open);
        },
        "name"_a, "closable"_a = false, "flags"_a = 0, "returns (expanded, opened)");

    m.def("End", &ImGui::End);

    m.def("Button", &ImGui::Button, "text"_a, "size"_a = ImVec2(0, 0));

    m.def(
        "Checkbox", [](const char* label, bool state) {
            return py::make_tuple(ImGui::Checkbox(label, &state), state);
        },
        "label"_a, "state"_a, "returns (clicked, state)");

    m.def(
        "Text", [](const char* text) { ImGui::Text("%s", text); }, "text"_a);

    m.def(
        "RadioButton", [](const char* label, bool state) -> bool {
            return ImGui::RadioButton(label, state);
        },
        "label"_a, "active"_a);

    m.def(
        "Combo", [](const char* label, int current, py::list items, int height_in_items) {
            py::str in_items = py::str("\0").attr("join")(items);

            return py::make_tuple(
                ImGui::Combo(label, &current, py::cast<std::string>(in_items).c_str(), height_in_items),
                current);
        },
        "label"_a, "current"_a, "items"_a, "height_in_items"_a = -1, "returns (changed, current)");

    m.def("BeginMenuBar", &ImGui::BeginMenuBar);

    m.def("EndMenuBar", &ImGui::EndMenuBar);

    m.def("BeginMenu", &ImGui::BeginMenu, "label"_a, "enabled"_a = true);

    m.def("EndMenu", &ImGui::EndMenu);

    m.def(
        "MenuItem", [](const char* label, const char* shortcut, bool selected, bool enabled) {
            return py::make_tuple(
                ImGui::MenuItem(label, shortcut, &selected, enabled),
                selected);
        },
        "label"_a, "shortcut"_a = "", "selected"_a = false, "enabled"_a = true, "returns (clicked, state)");

    m.def("Separator", &ImGui::Separator);

    m.def("LogToClipboard", &ImGui::LogToClipboard, "auto_open_depth"_a = -1);

    create_table_functions(m);
}