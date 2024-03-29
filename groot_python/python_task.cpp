#include "python_task.hpp"


py::object py_return(py::object obj)
{
    PyErr_SetObject(PyExc_StopIteration, obj.inc_ref().ptr());
    throw py::error_already_set();
}

#define PY_YIELD(obj) return static_cast<py::object>(obj);
#define PY_RETURN(obj) return py_return(static_cast<py::object>(obj));

void create_task_module(py::module& m)
{
    using arg = py::arg;

    py::enum_<TaskMode>(m, "TaskMode")
        .value("Async", TaskMode::Async)
        .value("Sync", TaskMode::Sync)
        .value("Inline", TaskMode::Inline);

    py::class_<PythonTask>(m, "Task")
        .def(py::init<py::object, TaskMode>(), arg("function"), arg("mode") = TaskMode::Sync)
        .def("then", &PythonTask::then_python,
            arg("task"),
            arg("TaskMode") = TaskMode::Sync,
            py::return_value_policy::reference_internal)
        .def("ready", &PythonTask::ready)
        .def("get", &PythonTask::get)
        .def(
            "__iter__", [](PythonTask& task) -> PythonTask& {
                return task;
            },
            py::return_value_policy::reference_internal)
        .def(
            "__await__", [](PythonTask& task) -> PythonTask& {
                return task;
            },
            py::return_value_policy::reference_internal)
        .def("__next__", [](PythonTask& task) {
            sync_scheduler().try_run_one_task();

            if (task.ready()) {
                PY_RETURN(task.get());
            } else {
                PY_YIELD(py::none());
            }
        });
}