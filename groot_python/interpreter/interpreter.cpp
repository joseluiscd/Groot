#include <Python.h>

extern "C" {

extern PyObject* PyInit_pygroot();

int main(int argc, char**argv)
{
    PyImport_AppendInittab("pygroot", PyInit_pygroot);
    Py_Initialize();
    Py_BytesMain(argc, argv);
}

}