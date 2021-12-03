#include "python_task.hpp"

thread_local PyThreadState* PythonThreadState::state = nullptr;

void PythonTask::run_till_completion()
{
    auto& sched = sync_scheduler();

    while (!this->ready()) {
        sched.run_all_tasks();
    }
}

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
        .def("then", &PythonTask::then_python, (arg("task"), arg("TaskMode") = TaskMode::Sync),return_self<>())
        .def("ready", &PythonTask::ready)
        .def("get", &PythonTask::get)
        .def("run_till_completion", &PythonTask::run_till_completion)
        ;
    
}