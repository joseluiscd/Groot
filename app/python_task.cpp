#include "python_task.hpp"

thread_local PythonThreadState ReleaseGil::thread_state = PythonThreadState();

void create_task_module()
{
    using namespace boost::python;

    object types = import("types");
    object module = types.attr("ModuleType");

    object task_mod = module("task", "Namespace for tasks");
    scope().attr("task") = task_mod;
    scope tasks(task_mod);

    enum_<TaskMode>("TaskMode")
        .value("Async", TaskMode::Async)
        .value("Sync", TaskMode::Sync)
        .value("Inline", TaskMode::Inline);

    class_<PythonTask, boost::noncopyable>("Task", init<object, str, TaskMode>((arg("function"), arg("name"), arg("mode") = TaskMode::Sync)))
        .def("then", &PythonTask::then_python, return_self<>())
        ;
}