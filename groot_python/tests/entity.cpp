#include <entity.hpp>
#include <registry.hpp>

extern "C" {
extern PyObject* PyInit_pygroot();
}

int main(int argc, char**argv)
{
    PyImport_AppendInittab("pygroot", PyInit_pygroot);
    Py_Initialize();
    Py_BytesMain(argc, argv);
}
/*
TEST_CASE("Python works")
{
    using namespace boost::python;
    PyImport_AppendInittab("pygroot", PyInit_pygroot);
    Py_Initialize();

    object main_module = import("__main__");
    object pygroot = import("pygroot");
    REQUIRE(pygroot != object());

    SUBCASE("registry")
    {
        object reg = eval("pygroot.Registry()", main_module);
        REQUIRE(extract<Registry>(reg).check());

        SUBCASE("create entity")
        {
            object e = eval("");
        }
    }
}
*/