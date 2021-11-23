#include "python_imgui.hpp"
#include <boost/python.hpp>
#include <gfx/imgui/imgui.h>

struct ImGuiContext {
};

void create_imgui_module()
{
    using namespace boost::python;

    class_<ImVec2>("ImVec2", init<float, float>())
        .def(init<>())
        .def_readwrite("x", &ImVec2::x)
        .def_readwrite("y", &ImVec2::y);

    class_<ImVec4>("ImVec4", init<float, float, float, float>())
        .def(init<>())
        .def_readwrite("x", &ImVec4::x)
        .def_readwrite("y", &ImVec4::y)
        .def_readwrite("z", &ImVec4::z)
        .def_readwrite("w", &ImVec4::w);

    class_<ImGuiContext>("ImGui", "ImGui context")
        .def(
            "Begin", +[](const char* name, bool closable, int flags) {
                bool open = true;

                return make_tuple(ImGui::Begin(name, closable ? &open : nullptr, flags), open);
            },
            (arg("name"), arg("closable") = false, arg("flags") = 0), "returns (expanded, opened)")
        .staticmethod("Begin")

        .def("End", &ImGui::End)
        .staticmethod("End")

        .def("Button", &ImGui::Button, (arg("text"), arg("size") = ImVec2(0, 0)))
        .staticmethod("Button")

        .def(
            "Checkbox", +[](const char* label, bool state) {
                return make_tuple(ImGui::Checkbox(label, &state), state);
            },
            (arg("label"), arg("state")), "returns (clicked, state)")
        .staticmethod("Checkbox")

        .def(
            "Text", +[](const char* text) { ImGui::Text("%s", text); }, (arg("text")))
        .staticmethod("Text")

        .def(
            "RadioButton", +[](const char* label, bool state) -> bool {
                return ImGui::RadioButton(label, state);
            },
            (arg("label"), arg("active")))
        .staticmethod("RadioButton")

        .def(
            "Combo", +[](const char* label, int current, list items, int height_in_items) {
                str in_items = str("\0").join(items);

                return make_tuple(
                    ImGui::Combo(label, &current, extract<const char*>(in_items), height_in_items),
                    current);
            },
            (arg("label"), arg("current"), arg("items"), arg("height_in_items") = -1), "returns (changed, current)")

        .def("BeginMenuBar", &ImGui::BeginMenuBar)
        .staticmethod("BeginMenuBar")

        .def("EndMenuBar", &ImGui::EndMenuBar)
        .staticmethod("EndMenuBar")

        .def("BeginMenu", &ImGui::BeginMenu, (arg("label"), arg("enabled") = true))
        .staticmethod("BeginMenu")

        .def("EndMenu", &ImGui::EndMenu)
        .staticmethod("EndMenu")

        .def(
            "MenuItem", +[](const char* label, const char* shortcut, bool selected, bool enabled) {
                return make_tuple(
                    ImGui::MenuItem(label, shortcut, &selected, enabled),
                    selected);
            },
            (arg("label"), arg("shortcut") = "", arg("selected") = false, arg("enabled") = true), "returns (clicked, state)")
        .staticmethod("MenuItem")

        .def("Separator", &ImGui::Separator)
        .staticmethod("Separator")

        /**/
        ;
}