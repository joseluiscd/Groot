#include "python_imgui.hpp"
#include <boost/python.hpp>
#include <gfx/imgui/imgui.h>

struct ImGuiContext {
};

void create_table_functions()
{
    using namespace boost::python;

    def("BeginTable", &ImGui::BeginTable,
        (
            arg("name"),
            arg("columns"),
            arg("flags") = 0,
            arg("size") = ImVec2(0.0, 0.0),
            arg("inner_width") = 0.0),
        "Begin a table");
    def("EndTable", &ImGui::EndTable);
    def("TableNextColumn", &ImGui::TableNextColumn);
    def("TableNextRow", &ImGui::TableNextRow, (arg("row_flags") = 0, arg("min_height") = 0.0f));
}

void create_imgui_module()
{
    using namespace boost::python;

    scope current_scope = class_<ImGuiContext>("ImGui", no_init);

    class_<ImVec2>("Vec2", init<float, float>())
        .def(init<>())
        .def_readwrite("x", &ImVec2::x)
        .def_readwrite("y", &ImVec2::y);

    class_<ImVec4>("Vec4", init<float, float, float, float>())
        .def(init<>())
        .def_readwrite("x", &ImVec4::x)
        .def_readwrite("y", &ImVec4::y)
        .def_readwrite("z", &ImVec4::z)
        .def_readwrite("w", &ImVec4::w);

    def(
        "Begin", +[](const char* name, bool closable, int flags) {
            bool open = true;

            return make_tuple(ImGui::Begin(name, closable ? &open : nullptr, flags), open);
        },
        (arg("name"), arg("closable") = false, arg("flags") = 0), "returns (expanded, opened)");

    def("End", &ImGui::End);

    def("Button", &ImGui::Button, (arg("text"), arg("size") = ImVec2(0, 0)));

    def(
        "Checkbox", +[](const char* label, bool state) {
            return make_tuple(ImGui::Checkbox(label, &state), state);
        },
        (arg("label"), arg("state")), "returns (clicked, state)");

    def(
        "Text", +[](const char* text) { ImGui::Text("%s", text); }, (arg("text")));

    def(
        "RadioButton", +[](const char* label, bool state) -> bool {
            return ImGui::RadioButton(label, state);
        },
        (arg("label"), arg("active")));

    def(
        "Combo", +[](const char* label, int current, list items, int height_in_items) {
            str in_items = str("\0").join(items);

            return make_tuple(
                ImGui::Combo(label, &current, extract<const char*>(in_items), height_in_items),
                current);
        },
        (arg("label"), arg("current"), arg("items"), arg("height_in_items") = -1), "returns (changed, current)");

    def("BeginMenuBar", &ImGui::BeginMenuBar);

    def("EndMenuBar", &ImGui::EndMenuBar);

    def("BeginMenu", &ImGui::BeginMenu, (arg("label"), arg("enabled") = true));

    def("EndMenu", &ImGui::EndMenu);

    def(
        "MenuItem", +[](const char* label, const char* shortcut, bool selected, bool enabled) {
            return make_tuple(
                ImGui::MenuItem(label, shortcut, &selected, enabled),
                selected);
        },
        (arg("label"), arg("shortcut") = "", arg("selected") = false, arg("enabled") = true), "returns (clicked, state)");

    def("Separator", &ImGui::Separator);
    
    create_table_functions();
}